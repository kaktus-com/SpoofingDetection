"""Inspect recorded external carrier phase with PX4 attitude and accelerometers.

GNSS location may be read only through an explicitly enabled geometry lookup;
it never supplies displacement. GPS-derived UTC headers are never used.
The receive-time anchor is provisional, not a recovered measurement clock. No
benign/spoofed likelihood is evaluated without geometry and calibration.
"""

import argparse
from collections import Counter
import csv
from dataclasses import dataclass, replace
import hashlib
import json
from pathlib import Path

import numpy as np

from .detector import DetectorConfig
from .geometry import quaternion_multiply, quaternion_to_matrix, rotate
from .motion import (ImuSeries, MotionUnavailable, estimate_antenna_displacement,
                     integrate_linear_acceleration, quaternion_at)
from .motion_evaluation import remove_polynomial
from .motion_uncertainty import assess_motion_noise

ULOG_TOPICS = ('sensor_combined', 'vehicle_attitude', 'vehicle_land_detected')
GPS_L5_WAVELENGTH_M = 299792458. / 1176450000.


def clean_json(value):
    if isinstance(value, dict):
        return {str(k): clean_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, np.ndarray)):
        return [clean_json(v) for v in value]
    if isinstance(value, (float, np.floating)):
        return float(value) if np.isfinite(value) else None
    if isinstance(value, np.integer):
        return int(value)
    if isinstance(value, np.bool_):
        return bool(value)
    return value


def write_json(path, value):
    Path(path).write_text(json.dumps(clean_json(value), indent=2, allow_nan=False)+'\n')


def write_csv(path, rows, fields=None):
    rows = list(rows)
    fields = fields or (list(rows[0]) if rows else [])
    with Path(path).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({k: ('' if v is None else v) for k, v in row.items()} for row in rows)


def summary(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return dict(zip(('min', 'median', 'p95', 'max'), np.percentile(a, [0, 50, 95, 100]))) if len(a) else None


@dataclass
class RecordedCarrier:
    tow: np.ndarray
    phase: np.ndarray
    signals: tuple
    host: np.ndarray
    attitude_host: np.ndarray
    paired_boot: np.ndarray
    paired_rpy_deg: np.ndarray
    malformed_rows: int = 0
    duplicate_rows: int = 0

    @property
    def relative_time(self):
        return self.tow-self.tow[0]


def load_recorded_carrier(path):
    """Read original wide/long CSV phase; reject conflicting duplicate epochs."""
    epochs, malformed, duplicates = {}, 0, 0
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            try:
                keys = ['gps_time', 'carrier_sync_time', 'imu_sync_time',
                        'mavlink_time_boot_ms', 'roll', 'pitch', 'yaw']
                values = np.array([float(row[k]) for k in keys])
                if not np.isfinite(values).all():
                    raise ValueError('nonfinite timestamp/attitude')
                if 'sat' in row:
                    phases = {row['sat']: float(row['carrier'])} if row['carrier'] else {}
                else:
                    phases = {k[8:].replace('_', ':'): float(v) for k, v in row.items()
                              if k.startswith('carrier_') and k != 'carrier_sync_time' and v}
                phases = {k: v for k, v in phases.items() if np.isfinite(v)}
            except (ValueError, TypeError, KeyError):
                malformed += 1
                continue
            tow = values[0]
            if tow in epochs:
                duplicates += 1
                previous = epochs[tow][1]
                if any(k in previous and previous[k] != v for k, v in phases.items()):
                    raise ValueError(f'Conflicting phase observations at GPS time {tow}')
                previous.update(phases)
            else:
                epochs[tow] = (values, phases)
    if len(epochs) < 2:
        raise ValueError('Need at least two distinct saved carrier epochs')
    times = np.array(sorted(epochs))
    rows = [epochs[t][0] for t in times]
    signals = tuple(sorted(set().union(*(epochs[t][1] for t in times))))
    if not signals:
        raise ValueError('All saved carrier columns are empty')
    a = np.array(rows)
    phase = np.array([[epochs[t][1].get(s, np.nan) for s in signals] for t in times])
    return RecordedCarrier(times, phase, signals, a[:, 1], a[:, 2], a[:, 3]/1000,
                           a[:, 4:7], malformed, duplicates)


def clock_diagnostics(carrier):
    t = carrier.relative_time
    # Add host time difference to the paired ATTITUDE message timestamp. This
    # is a receive-time proxy; message age/transport delay remains unknown.
    proxy = carrier.paired_boot + carrier.host-carrier.attitude_host
    anchor = float(np.min(proxy-t))
    return {'status': 'unverified', 'rate_model': 'GPS and PX4 clocks assumed to advance at 1 second/second',
            'anchor_boot_s': anchor, 'anchor_method': 'minimum(receive-time PX4 proxy - GPS elapsed time)',
            'equation': 'candidate_boot_s = anchor_boot_s + (gps_tow_s - first_gps_tow_s) + offset_s',
            'gps_elapsed_s': float(t[-1]), 'recorder_elapsed_s': float(carrier.host[-1]-carrier.host[0]),
            'receive_delay_growth_s': float(carrier.host[-1]-carrier.host[0]-t[-1]),
            'paired_boot_elapsed_s': float(carrier.paired_boot[-1]-carrier.paired_boot[0]),
            'saved_pair_offset_s': summary(carrier.host-carrier.attitude_host),
            'paired_boot_nonincreasing_steps': int(np.count_nonzero(np.diff(carrier.paired_boot) <= 0)),
            'gps_derived_ulog_utc_used': False,
            'interpretation': 'Removing variable receive delay does not recover absolute sensor latency. No offset is certified or chosen from detector outcomes.'}


def supplied_attitude_on_accel(accel_times, attitude_times, quaternions, reset_counters,
                               max_attitude_gap_s=.075):
    """Interpolate supplied PX4 attitude, masking coverage, gaps and resets."""
    t = np.asarray(accel_times)
    a = np.asarray(attitude_times)
    if len(a) < 2 or np.any(np.diff(a) <= 0):
        raise ValueError('Attitude sample times must increase strictly')
    right = np.searchsorted(a, t, side='left')
    covered = (t >= a[0]) & (t <= a[-1])
    right = np.clip(right, 0, len(a)-1)
    exact = np.abs(a[right]-t) < 1e-10
    left = np.where(exact, right, np.maximum(0, right-1))
    gap = a[right]-a[left]
    reset = np.asarray(reset_counters)
    valid = covered & (gap <= max_attitude_gap_s) & (reset[left] == reset[right])
    q = np.full((len(t), 4), np.nan)
    if np.any(valid):
        q[valid] = quaternion_at(a, np.asarray(quaternions), t[valid])
    return q, valid


def load_px4_imu(path, lever_arm=None):
    # Kept local so synthetic-only workflows do not require pyulog.
    from pyulog import ULog
    ulog = ULog(str(path), message_name_filter_list=list(ULOG_TOPICS))
    def topic(name):
        return next((ds.data for ds in ulog.data_list if ds.name == name and ds.multi_id == 0), None)
    acc, att, land = topic('sensor_combined'), topic('vehicle_attitude'), topic('vehicle_land_detected')
    if acc is None or att is None:
        raise ValueError('ULog requires sensor_combined and vehicle_attitude')
    relative = np.asarray(acc['accelerometer_timestamp_relative'], dtype=float)
    times = (np.asarray(acc['timestamp'], dtype=float)+relative)/1e6
    at = np.asarray(att['timestamp_sample'], dtype=float)/1e6
    q_ned = np.column_stack([att[f'q[{i}]'] for i in range(4)])
    q_enu = quaternion_multiply(np.array([0., np.sqrt(.5), np.sqrt(.5), 0.]), q_ned)
    q, valid = supplied_attitude_on_accel(times, at, q_enu, att['quat_reset_counter'])
    force = np.column_stack([acc[f'accelerometer_m_s2[{i}]'] for i in range(3)])
    gyro = np.column_stack([acc[f'gyro_rad[{i}]'] for i in range(3)])
    valid &= (relative != 2147483647) & np.isfinite(force).all(axis=1) & np.isfinite(gyro).all(axis=1)
    valid &= np.asarray(acc['accelerometer_clipping']) == 0
    valid &= np.r_[True, np.diff(times) > 0]
    if np.count_nonzero(valid) < 2:
        raise ValueError('No usable accelerometer samples with supplied attitude')
    nominal = float(1/np.median(np.diff(times)[np.diff(times) > 0]))
    lever = np.zeros(3) if lever_arm is None else np.asarray(lever_arm, dtype=float)
    imu = ImuSeries(times[valid], force[valid], gyro[valid], q[valid], lever,
                    np.array([0., 0., -9.80665]), nominal, accel_noise_sigma_m_s2=1.)
    resets = at[np.r_[False, np.diff(att['quat_reset_counter'].astype(int)) != 0]]
    landed = np.zeros(len(imu.time_s), dtype=bool)
    if land is not None:
        lt = np.asarray(land['timestamp'], float)/1e6
        li = np.searchsorted(lt, imu.time_s, side='right')-1
        covered = li >= 0
        landed[covered] = np.asarray(land['landed'], bool)[li[covered]]
    # A stationary proxy, not a calibrated in-flight white-noise measurement.
    dforce = np.diff(imu.specific_force_body_m_s2, axis=0)
    pairs = landed[1:] & landed[:-1] & (np.diff(imu.time_s) < 1.5/nominal)
    proxy = None
    if np.count_nonzero(pairs) >= 20:
        delta = dforce[pairs]
        proxy = np.median(np.abs(delta-np.median(delta, axis=0)), axis=0)/.9538725524
    metadata = {'topics_read': list(ULOG_TOPICS), 'gps_position_or_utc_header_used': False,
                'supplied_attitude': 'PX4 vehicle_attitude.q FRD→NED, converted to FRD→ENU',
                'specific_force': 'sensor_combined FRD m/s², already sensor-calibrated; no second board rotation',
                'gyro_attitude_estimation': False, 'extra_ekf_bias_correction': False,
                'nominal_imu_rate_hz': nominal, 'input_accel_samples': len(times),
                'usable_attitude_accel_samples': len(imu.time_s), 'masked_samples': int(np.count_nonzero(~valid)),
                'max_attitude_interpolation_gap_s': .075, 'accel_gap_summary_s': summary(np.diff(times)),
                'attitude_gap_summary_s': summary(np.diff(at)), 'attitude_reset_boot_s': resets,
                'ground_noise_proxy_axis_m_s2': proxy, 'ground_noise_proxy_pairs': int(np.count_nonzero(pairs)),
                'noise_proxy_interpretation': 'MAD of differences / 0.95387 on adjacent landed samples; vibration, colour and sensor filtering are not calibrated',
                'lever_arm_body_m': None if lever_arm is None else lever,
                'displacement_reference': 'IMU only: antenna lever arm unknown' if lever_arm is None else 'antenna: user-supplied lever arm',
                'start_boot_s': float(imu.time_s[0]), 'end_boot_s': float(imu.time_s[-1])}
    return imu, metadata


def phase_windows(carrier, width_s=1.):
    """Time-aware polynomial residuals; gaps are recorded, never filled."""
    relative = carrier.relative_time
    windows = []
    for index, start in enumerate(np.arange(0., relative[-1], width_s)):
        ids = np.flatnonzero((relative >= start-1e-8) & (relative < start+width_s-1e-8))
        if not len(ids):
            continue
        t = relative[ids]
        phase = carrier.phase[ids]
        residual = np.full(phase.shape, np.nan)
        for j in range(len(carrier.signals)):
            ok = np.isfinite(phase[:, j])
            if np.count_nonzero(ok) >= 4:
                y = phase[ok, j]-phase[ok, j][0]
                residual[ok, j] = remove_polynomial(t[ok], y[:, None])[:, 0]
        common = np.isfinite(phase).all(axis=0)
        gap = float(np.max(np.diff(t))) if len(t) > 1 else None
        windows.append({'index': index, 'start_s': float(start), 'end_s': float(start+width_s),
                        'indices': ids, 'time_s': t, 'phase_residual_cycles': residual,
                        'samples': len(ids), 'common_signals': int(common.sum()),
                        'common_signal_ids': [s for s, ok in zip(carrier.signals, common) if ok],
                        'max_epoch_gap_s': gap,
                        'phase_rms_cycles': [float(np.sqrt(np.mean(c[np.isfinite(c)]**2))) if np.isfinite(c).any() else None for c in residual.T],
                        'availability_pass': bool(len(t) >= 10 and common.sum() >= 4 and gap is not None and gap <= .075+1e-9),
                        'continuity_status': 'lock, half-cycle and receiver validity flags not saved; continuity unverified'})
    return windows


def timing_profile(carrier, imu, windows, anchor, reset_times):
    """Discovery/holdout consistency probe with unknown LOS, not a detector.

    One unconstrained direction per signal is learned across discovery windows
    and used unchanged on holdout windows. Trend removal uses each window's
    timestamps. Neither curve certifies the physical time mapping.
    """
    position, _ = integrate_linear_acceleration(imu.time_s, imu.linear_acceleration_enu_m_s2)
    position += rotate(quaternion_to_matrix(imu.quaternion_body_to_enu_wxyz), imu.lever_arm_body_m)
    offsets = np.linspace(-5., 1., 121)
    # Fixed central phase windows for every offset: no moving validation set.
    selected = [w for w in windows if w['samples'] >= 10
                and anchor+w['time_s'][0]-5 >= imu.time_s[0]
                and anchor+w['time_s'][-1]+1 <= imu.time_s[-1]
                and not any(anchor+w['time_s'][0]-5 <= r <= anchor+w['time_s'][-1]+1 for r in reset_times)]
    selected = [w for w in selected if not np.any(np.diff(imu.time_s[
        max(0, np.searchsorted(imu.time_s, anchor+w['time_s'][0]-5)-1):
        np.searchsorted(imu.time_s, anchor+w['time_s'][-1]+1)+1]) > .02)]
    discovery = [w for i, w in enumerate(selected) if i % 2 == 0]
    heldout = [w for i, w in enumerate(selected) if i % 2 == 1]
    def matrices(subset, offset, signal):
        designs, targets = [], []
        for w in subset:
            ids = w['indices']
            if not np.isfinite(carrier.phase[ids, signal]).all():
                continue
            query = anchor+w['time_s']+offset
            lo = max(0, np.searchsorted(imu.time_s, query[0])-1)
            hi = min(len(imu.time_s), np.searchsorted(imu.time_s, query[-1])+1)
            if np.any(np.diff(imu.time_s[lo:hi]) > .02):
                continue
            p = np.column_stack([np.interp(query, imu.time_s, position[:, j]) for j in range(3)])
            designs.append(remove_polynomial(query, p))
            targets.append(w['phase_residual_cycles'][:, signal])
        return (np.vstack(designs), np.concatenate(targets)) if designs else (None, None)
    rows = []
    for offset in offsets:
        train, validation, norms = [], [], []
        for j in range(len(carrier.signals)):
            d, y = matrices(discovery, offset, j)
            v, target = matrices(heldout, offset, j)
            if d is None or v is None or len(y) < 20 or len(target) < 20:
                continue
            beta = np.linalg.lstsq(d, y, rcond=1e-6)[0]
            train.append(1-np.sum((y-d@beta)**2)/max(np.sum(y*y), 1e-30))
            validation.append(1-np.sum((target-v@beta)**2)/max(np.sum(target*target), 1e-30))
            norms.append(np.linalg.norm(beta)*GPS_L5_WAVELENGTH_M)
        rows.append({'offset_s': float(offset), 'discovery_r2': float(np.median(train)) if train else None,
                     'holdout_r2': float(np.median(validation)) if validation else None,
                     'median_fitted_direction_norm': float(np.median(norms)) if norms else None})
    finite = [r for r in rows if r['discovery_r2'] is not None and r['holdout_r2'] is not None]
    best_train = max(finite, key=lambda r: r['discovery_r2']) if finite else None
    best_test = max(finite, key=lambda r: r['holdout_r2']) if finite else None
    return {'method': 'alternating fixed 1 s discovery/holdout windows; shared unconstrained response per signal',
            'score': 'median across signals of 1 - squared prediction error / quadratic-removed phase energy',
            'discovery_window_indices': [w['index'] for w in discovery],
            'holdout_window_indices': [w['index'] for w in heldout],
            'best_discovery_offset_s': None if best_train is None else best_train['offset_s'],
            'best_holdout_offset_s': None if best_test is None else best_test['offset_s'],
            'rows': rows, 'status': 'diagnostic_only_not_clock_validation',
            'limitations': 'Supplied lever arm included; coloured vibration and attitude/latency errors unmodelled. Unconstrained fitted directions may be nonphysical. Gaps up to 20 ms permitted only for this probe. No gamma, significance test or auto-selected clock offset.'}


def inspect_windows(carrier, imu, windows, clock, metadata, offset_s, noise_sigmas):
    output = []
    config = DetectorConfig()
    for w in windows:
        record = dict(w)
        query = clock['anchor_boot_s']+w['time_s']+offset_s
        reasons = ['measurement_clock_unverified', 'satellite_geometry_missing',
                   'receiver_quality_flags_missing', 'sensor_noise_model_uncalibrated']
        if metadata['lever_arm_body_m'] is None:
            reasons.append('antenna_lever_arm_unknown')
        if not w['availability_pass']:
            reasons.append('phase_availability_or_gap')
        record.update(candidate_boot_s=query, decision='not_evaluated', gamma=None,
                      readiness_reasons=reasons, integration_status='not_attempted', noise_sensitivity=[])
        try:
            if any(query[0] <= r <= query[-1] for r in metadata['attitude_reset_boot_s']):
                raise MotionUnavailable('px4_attitude_reset')
            estimate = estimate_antenna_displacement(imu, query)
            observable = remove_polynomial(query, estimate.antenna_displacement_m)
            singular = np.linalg.svd(observable, compute_uv=False)/np.sqrt(len(query))
            for sigma in noise_sigmas:
                varied = replace(estimate, displacement_covariance_m2=estimate.displacement_covariance_m2*sigma**2,
                                 observable_covariance_m2=estimate.observable_covariance_m2*sigma**2,
                                 diagnostics={**estimate.diagnostics, 'accel_noise_sigma_m_s2': sigma})
                check = assess_motion_noise(varied, config)
                record['noise_sensitivity'].append({'sigma_m_s2': sigma, **check})
            record.update(integration_status='integrated', displacement_m=estimate.antenna_displacement_m,
                          observable_displacement_m=observable, imu_displacement_m=estimate.imu_displacement_m,
                          lever_displacement_m=estimate.lever_displacement_m,
                          observed_axis_rms_m=singular,
                          observed_total_rms_m=float(np.linalg.norm(observable)/np.sqrt(len(query))),
                          integration_diagnostics=estimate.diagnostics)
        except MotionUnavailable as error:
            record['integration_status'] = str(error)
            reasons.append(str(error))
        output.append(record)
    return output


def run(recording_dir, ulog_path, output_dir, clock_offset_s=0., lever_arm=None,
        noise_sigmas=(.3, 1., 3.), plots=True, expected_case='unknown',
        navigation_file=None, origin_from_gnss=False, estimate_clock=False):
    recording_dir, ulog_path, output_dir = map(Path, (recording_dir, ulog_path, output_dir))
    if not np.isfinite(clock_offset_s) or any(not np.isfinite(s) or s <= 0 for s in noise_sigmas):
        raise ValueError('Clock offset must be finite and sensitivity sigmas must be positive')
    if lever_arm is not None and (np.shape(lever_arm) != (3,) or not np.isfinite(lever_arm).all()):
        raise ValueError('Lever arm requires three finite FRD metres')
    source = recording_dir/'synced_measurements.csv'
    for path in (source, ulog_path):
        if output_dir.resolve() in path.resolve().parents:
            raise ValueError('Output directory must not contain original source recordings')
    carrier = load_recorded_carrier(source)
    if any(not s.startswith('0:') or not s.endswith(':7') for s in carrier.signals):
        raise ValueError('This first adapter requires GPS L5_Q signals; do not silently assign a wavelength to other bands')
    imu, metadata = load_px4_imu(ulog_path, lever_arm)
    clock = clock_diagnostics(carrier)
    windows = phase_windows(carrier)
    print('Loaded raw external phases and supplied PX4 attitude/accelerometers.', flush=True)
    profile = timing_profile(carrier, imu, windows, clock['anchor_boot_s'], metadata['attitude_reset_boot_s'])
    if estimate_clock:
        if profile['best_discovery_offset_s'] is None:
            raise ValueError('No discovery offset could be estimated')
        clock_offset_s = profile['best_discovery_offset_s']
    records = inspect_windows(carrier, imu, windows, clock, metadata, clock_offset_s, noise_sigmas)
    geometry = None
    if navigation_file is not None:
        if not origin_from_gnss:
            raise ValueError('Navigation geometry needs an explicitly authorized origin lookup')
        from datetime import datetime
        from .broadcast_geometry import (origin_from_ulog, read_gps_navigation,
                                         nominal_geometry, GPS_EPOCH)
        frame = origin_from_ulog(ulog_path)
        nav = read_gps_navigation(navigation_file)
        week = int((datetime.strptime(recording_dir.name[:8], '%Y%m%d')-GPS_EPOCH).total_seconds()//604800)
        geometry = {'origin': {'latitude_deg':frame.latitude_deg, 'longitude_deg':frame.longitude_deg, 'ellipsoid_height_m':frame.height_m},
                    'origin_source':'user-authorized median ULog GNSS location ONLY; not displacement or velocity',
                    'navigation_file':str(Path(navigation_file).resolve()),
                    'archived_navigation_file':'adapted_inputs/broadcast_navigation.rnx.gz',
                    'navigation_sha256':hashlib.sha256(Path(navigation_file).read_bytes()).hexdigest(),
                    'orbit_reference':'https://www.gps.gov/sites/default/files/2025-07/IS-GPS-200N.pdf',
                    'navigation_source':'https://noaa-cors-pds.s3.amazonaws.com/rinex/2026/188/brdc1880.26n.gz',
                    'navigation_documentation':'https://www.ngs.noaa.gov/CORS/data.shtml',
                    'reference':nominal_geometry(nav,carrier.signals,float(np.median(carrier.tow)),frame,gps_week=week)}
        for r in records:
            rows = nominal_geometry(nav,carrier.signals,float(carrier.tow[0]+np.mean(r['time_s'])),frame,gps_week=week)
            r['geometry'] = rows
            r['readiness_reasons'].remove('satellite_geometry_missing')
            if r['integration_status']=='integrated':
                los=np.array([[a[k] for k in ['nominal_los_enu_east','nominal_los_enu_north','nominal_los_enu_up']] for a in rows])
                prediction = -r['displacement_m']@los.T/GPS_L5_WAVELENGTH_M
                r['nominal_phase_cycles'] = remove_polynomial(r['candidate_boot_s'],prediction)
                r['nominal_phase_interpretation'] = 'projection in nominal ENU; no fitted common attitude-frame rotation; not a benign/spoofed likelihood decision'
    # Conditional replay stops at existing motion-noise/gap gates. Unknown
    # receiver lock flags are not replaced with fabricated measurements.
    for r in records:
        r['conditional_screen'] = 'inconclusive'
        r['conditional_screen_reason'] = (r['integration_status'] if r['integration_status']!='integrated'
            else 'phase_availability_or_gap' if not r['availability_pass']
            else 'motion_below_noise_floor' if all(not c['passed'] for c in r['noise_sensitivity'])
            else 'required_field_calibration_or_quality_unverified')
    output_dir.mkdir(parents=True, exist_ok=True)
    inputs = output_dir/'adapted_inputs'
    inputs.mkdir(exist_ok=True)
    meta = {'recording': recording_dir.name, 'input_csv': str(source.resolve()), 'ulog': str(ulog_path.resolve()),
            'source_sha256': {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest() for p in (source, ulog_path)},
            'clock': clock, 'selected_offset_s': clock_offset_s,
            'offset_status': 'estimated_from_discovery_only_not_externally_verified' if estimate_clock else 'selected_sensitivity_not_verified' if clock_offset_s else 'provisional_receive_anchor_no_auto_selection',
            'px4': metadata, 'signals': carrier.signals, 'wavelength_m': GPS_L5_WAVELENGTH_M,
            'epochs': len(carrier.tow), 'phase_duration_s': float(carrier.relative_time[-1]),
            'malformed_rows': carrier.malformed_rows, 'duplicate_epoch_rows': carrier.duplicate_rows,
            'windows': len(records), 'integration_status_counts': dict(Counter(r['integration_status'] for r in records)),
            'availability_pass_windows': sum(r['availability_pass'] for r in records),
            'detector_evaluated_windows': 0, 'conclusive_decisions': 0,
            'conditional_quality_screen':dict(Counter(r['conditional_screen_reason'] for r in records)),
            'geometry':geometry,
            'blockers': ['verified measurement clock']+([] if geometry else ['nominal satellite LOS geometry'])+[
                         'receiver lock/validity/slip metadata or documented quality policy', 'measured noise model']
                         + ([] if lever_arm is not None else ['measured IMU-to-external-antenna offset']),
            'noise_sensitivity_sigmas_m_s2': noise_sigmas,
            'labels': {'expected_case': expected_case, 'source': 'user-declared flight condition' if expected_case != 'unknown' else 'not supplied',
                       'used_for_alignment_or_model_fit': False, 'old_detection_events_used': False},
            'exports': 'inspection inputs; missing flags and calibration remain null/blank, not synthesized',
            'model': 'IMU integration with supplied PX4 attitude; zero initial p/v each window; no gyro AHRS or fused position',
            'phase_quality': 'raw cycles; polynomial residuals use actual irregular times; missing phase not interpolated'}
    write_json(output_dir/'run.json', meta)
    write_json(output_dir/'clock_profile.json', profile)
    write_json(output_dir/'window_details.json', records)
    if geometry:
        write_json(output_dir/'satellite_geometry.json',geometry)
        write_csv(output_dir/'satellite_geometry.csv',geometry['reference'])
        import shutil
        archived = inputs/'broadcast_navigation.rnx.gz'
        if Path(navigation_file).resolve() != archived.resolve():
            shutil.copyfile(navigation_file,archived)
    write_csv(output_dir/'clock_profile.csv', profile['rows'])
    epoch_rows = [{'gps_tow_s': float(t), 'gps_elapsed_s': float(t-carrier.tow[0]),
                   'candidate_px4_boot_s': float(clock['anchor_boot_s']+t-carrier.tow[0]+clock_offset_s),
                   'carrier_receive_host_s': float(carrier.host[i]), 'paired_attitude_boot_s': float(carrier.paired_boot[i]),
                   'paired_attitude_host_s': float(carrier.attitude_host[i]),
                   'relative_receive_delay_s': float(carrier.host[i]-carrier.host[0]-(t-carrier.tow[0])),
                   'available_signals': int(np.isfinite(carrier.phase[i]).sum()), 'clock_verified': False}
                  for i, t in enumerate(carrier.tow)]
    write_csv(output_dir/'epoch_clock.csv', epoch_rows)
    write_csv(inputs/'carrier.csv', ({'gps_tow_s': float(t), 'candidate_px4_boot_s': epoch_rows[i]['candidate_px4_boot_s'],
                                    'signal_id': s, 'carrier_phase_cycles': float(carrier.phase[i, j]),
                                    'wavelength_m': GPS_L5_WAVELENGTH_M, 'receiver_valid': None,
                                    'lock_time_s': None, 'half_cycle_valid': None, 'sigma_cycles': None}
                                   for i, t in enumerate(carrier.tow) for j, s in enumerate(carrier.signals)
                                   if np.isfinite(carrier.phase[i, j])))
    imu_rows = []
    for i, t in enumerate(imu.time_s):
        row = {'px4_boot_s': float(t)}
        for prefix, array, axes in [('specific_force_body', imu.specific_force_body_m_s2, ['forward_m_s2','right_m_s2','down_m_s2']),
                                    ('angular_rate_body', imu.angular_rate_body_rad_s, ['forward_rad_s','right_rad_s','down_rad_s']),
                                    ('q_body_to_enu', imu.quaternion_body_to_enu_wxyz, ['w','x','y','z'])]:
            row.update({f'{prefix}_{axis}': float(array[i, j]) for j, axis in enumerate(axes)})
        imu_rows.append(row)
    write_csv(inputs/'imu.csv', imu_rows)
    write_json(inputs/'frame.json', {'body_axes': 'Forward, Right, Down', 'local_axes': 'East, North, Up',
                                     'quaternion': 'Hamilton wxyz, active FRD body to ENU',
                                     'gravity_enu_m_s2': [0.,0.,-9.80665],
                                     'lever_arm_body_m': metadata['lever_arm_body_m'],
                                     'status': 'inspection_only_not_replay_ready'})
    flat = []
    displacement_rows = []
    for r in records:
        row = {k: r.get(k) for k in ['index','start_s','end_s','samples','common_signals','max_epoch_gap_s',
                                    'availability_pass','integration_status','observed_total_rms_m','decision','gamma']}
        row['readiness_reasons'] = ';'.join(r['readiness_reasons'])
        row['conditional_screen_reason'] = r['conditional_screen_reason']
        for check in r['noise_sensitivity']:
            row[f"noise_check_sigma_{check['sigma_m_s2']:g}"] = check['passed']
        flat.append(row)
        if r['integration_status'] == 'integrated':
            for i, t in enumerate(r['time_s']):
                drow = {'window_index': r['index'], 'gps_elapsed_s': t, 'candidate_px4_boot_s': r['candidate_boot_s'][i]}
                for prefix, a in [('displacement',r['displacement_m']),('quadratic_removed',r['observable_displacement_m'])]:
                    drow.update({f'{prefix}_{axis}_m': float(a[i,j]) for j, axis in enumerate(['east','north','up'])})
                displacement_rows.append(drow)
    fields = list(dict.fromkeys(k for r in flat for k in r))
    write_csv(output_dir/'windows.csv', [{k: r.get(k) for k in fields} for r in flat], fields)
    write_csv(output_dir/'displacement.csv', displacement_rows,
              ['window_index','gps_elapsed_s','candidate_px4_boot_s',*[f'{p}_{a}_m' for p in ['displacement','quadratic_removed'] for a in ['east','north','up']]])
    from .recorded_flight_report import write_report
    write_report(output_dir, meta, carrier, imu, records, profile, plots)
    print(f"Saved {output_dir/'report.html'}; {meta['integration_status_counts']}; detector not evaluated.", flush=True)
    return meta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    base = Path('/home/borna/Documents/stored_flights')
    parser.add_argument('--recording-dir', type=Path, default=base/'extra_gps/recordings/20260707_152258')
    parser.add_argument('--ulog', type=Path, default=base/'flights/log_170_2026-7-7-15-25-22.ulg')
    parser.add_argument('--output-dir', type=Path, default=Path(__file__).parent/'results/recorded_july07')
    parser.add_argument('--clock-offset-s', type=float, default=0., help='Sensitivity offset from provisional receive anchor; never marks clock verified')
    parser.add_argument('--lever-arm-body', type=float, nargs=3, metavar=('FORWARD','RIGHT','DOWN'))
    parser.add_argument('--noise-sigmas', type=float, nargs='+', default=[.3,1.,3.])
    parser.add_argument('--expected-case', choices=['benign','spoofed','unknown'], default='unknown', help='User-declared evaluation label only; never used for timing or fitting')
    parser.add_argument('--navigation-file',type=Path)
    parser.add_argument('--origin-from-ulog',action='store_true',help='Explicit authorization: use ULog GNSS position only as approximate geometry origin')
    parser.add_argument('--estimate-clock',action='store_true',help='Use discovery-only timing estimate, still marked unverified')
    parser.add_argument('--no-plots', action='store_true')
    args = parser.parse_args()
    run(args.recording_dir, args.ulog, args.output_dir, args.clock_offset_s,
        args.lever_arm_body, tuple(args.noise_sigmas), not args.no_plots, args.expected_case,
        args.navigation_file,args.origin_from_ulog,args.estimate_clock)


if __name__ == '__main__':
    main()
