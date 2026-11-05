"""Offline interactive recording inspection and standalone scientific figures."""

import base64
from html import escape
import json
from pathlib import Path

import numpy as np

from .geometry import quaternion_to_matrix
from .recorded_flight import clean_json


def make_plots(output, meta, carrier, imu, records, profile):
    from .detector_plots import plt, setup_style
    from matplotlib.backends.backend_pdf import PdfPages
    from scipy.spatial.transform import Rotation
    setup_style()
    folder = output/'plots'
    folder.mkdir(exist_ok=True)
    names = []
    with PdfPages(output/'flight_inspection.pdf') as pdf:
        def save(fig, name):
            path = Path('plots')/(name+'.png')
            fig.savefig(output/path, dpi=150)
            pdf.savefig(fig)
            plt.close(fig)
            names.append(path)
        t = carrier.relative_time
        delay = carrier.host-carrier.host[0]-t
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), layout='constrained')
        axes[0].plot(t, carrier.host-carrier.host[0], label='Recorded arrival time')
        axes[0].plot(t, t, '--', label='External receiver epoch time')
        axes[0].set(xlabel='External GPS elapsed seconds', ylabel='Elapsed seconds')
        axes[0].legend()
        axes[1].plot(t, delay, color='#c47725')
        axes[1].set(xlabel='External GPS elapsed seconds', ylabel='Relative arrival delay (s)', title='Delay growth is removed from the candidate time axis; absolute latency remains unknown')
        for key, label, color in [('discovery_r2','Discovery','#255fa0'),('holdout_r2','Holdout','#a64b33')]:
            axes[2].plot([r['offset_s'] for r in profile['rows']], [r[key] for r in profile['rows']], label=label, color=color)
        axes[2].set(xlabel='Offset from provisional clock anchor (s)', ylabel='Median phase prediction R²', title='Exploratory shared-response check; no clock offset is certified')
        axes[2].legend()
        fig.suptitle('July recording · clock diagnosis and discovery/holdout check')
        save(fig, 'clock_alignment')

        fig, axes = plt.subplots(3, 1, figsize=(12, 10), layout='constrained')
        first = imu.time_s[0]
        x = imu.time_s-first
        # Convert ENU rotations back to NED only for familiar aerospace Euler plots.
        enu_from_ned = np.array([[0.,1.,0.],[1.,0.,0.],[0.,0.,-1.]])
        ned_from_body = np.einsum('ij,njk->nik', enu_from_ned.T, quaternion_to_matrix(imu.quaternion_body_to_enu_wxyz))
        rpy = Rotation.from_matrix(ned_from_body).as_euler('xyz', degrees=True)
        rpy[:, 2] = np.degrees(np.unwrap(np.radians(rpy[:, 2])))
        for j, label in enumerate(['Roll','Pitch','Yaw (unwrapped)']):
            axes[0].plot(x, rpy[:, j], label=label, lw=1)
        for j, label in enumerate(['Forward','Right','Down']):
            axes[1].plot(x, imu.specific_force_body_m_s2[:,j], label=label, lw=.55)
        for j, label in enumerate(['East','North','Up']):
            axes[2].plot(x, imu.linear_acceleration_enu_m_s2[:,j], label=label, lw=.55)
        for ax in axes:
            ax.legend(ncol=3)
            ax.set_xlabel(f'PX4 boot time − {first:.6f} s')
            for r in meta['px4']['attitude_reset_boot_s']:
                ax.axvline(r-first, color='#a64b33', ls='--', lw=1)
        axes[0].set_ylabel('Degrees')
        axes[1].set_ylabel('Body specific force (m/s²)')
        axes[2].set_ylabel('Gravity-removed ENU (m/s²)')
        fig.suptitle('Recorded PX4 attitude and accelerometers · reset marked; attitude supplied directly')
        save(fig, 'px4_motion')

        fig, axes = plt.subplots(3, 1, figsize=(12, 10), layout='constrained')
        for j, signal in enumerate(carrier.signals):
            ok = np.isfinite(carrier.phase[:, j])
            axes[0].plot(t[ok], carrier.phase[ok,j]-carrier.phase[ok,j][0], label=signal, lw=1)
            for r in records:
                yy = r['phase_residual_cycles'][:,j]
                axes[1].plot(r['time_s'], yy, '.', ms=1.5, color=f'C{j%10}')
            axes[2].scatter(t[ok], np.full(ok.sum(),j), marker='|', s=6)
        axes[0].legend(ncol=4, fontsize=8)
        axes[0].set_ylabel('Phase − first value (cycles)')
        axes[1].set_ylabel('Per-window residual (cycles)')
        axes[1].set_title('Each 1 s window removes its own quadratic; residual is not pure movement or receiver noise')
        axes[2].set(yticks=range(len(carrier.signals)), yticklabels=carrier.signals, ylabel='Signal availability')
        for ax in axes:
            ax.set_xlabel('External GPS elapsed seconds')
        fig.suptitle('External receiver · original GPS L5_Q carrier cycles · gaps retained')
        save(fig, 'carrier_quality')

        if meta.get('geometry'):
            fig = plt.figure(figsize=(9,8),layout='constrained')
            ax = fig.add_subplot(111,projection='polar')
            ax.set_theta_zero_location('N');ax.set_theta_direction(-1)
            ax.set_ylim(0,90);ax.set_yticks([0,30,60,90],['90°','60°','30°','0°'])
            for j,r in enumerate(meta['geometry']['reference']):
                angle=np.radians(r['azimuth_deg']);radius=90-r['elevation_deg']
                ax.scatter(angle,radius,s=95,color=f'C{j%10}')
                ax.annotate(r['signal_id'],(angle,radius),xytext=(7,7),textcoords='offset points',fontsize=10)
            fig.suptitle('GPS sky directions · independent broadcast ephemerides\nULog GNSS used only for the fixed approximate receiver location')
            save(fig,'satellite_sky')

        available = [r for r in records if r['integration_status']=='integrated']
        fig, axes = plt.subplots(2, 1, figsize=(12, 8), layout='constrained')
        if available:
            wx = [r['start_s'] for r in available]
            axes[0].plot(wx, [r['observed_total_rms_m']*1000 for r in available], '.-', label='Observed total RMS')
            axes[1].plot(wx, [r['observed_axis_rms_m'][-1]*1000 for r in available], '.-', label='Observed weakest-axis RMS')
            for sigma in meta['noise_sensitivity_sigmas_m_s2']:
                floor = []
                for r in available:
                    c = next(c for c in r['noise_sensitivity'] if c['sigma_m_s2']==sigma)
                    floor.append((c['noise_bound_rms_m']+c['min_axis_required_m'])*1000)
                axes[1].plot(wx, floor, '--', label=f'White-noise bound + axis threshold, σ={sigma:g} m/s²')
        for ax in axes:
            ax.set(xlabel='External GPS elapsed seconds (candidate alignment)', ylabel='Quadratic-removed displacement (mm)')
            ax.legend(fontsize=8)
        axes[0].set_title('Unknown initial p/v are set to zero in each independent window')
        axes[1].set_title('Sensitivity only: these white-noise levels are not an in-flight calibration')
        fig.suptitle('Recorded IMU → antenna displacement · provisional measurement clock')
        save(fig, 'motion_quality')
        if available:
            example = max(available, key=lambda r:r['observed_total_rms_m'] if r['availability_pass'] else -1)
            fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
            x = example['time_s']-example['time_s'][0]
            for j, label in enumerate(['East','North','Up']):
                axes[0,0].plot(x, example['displacement_m'][:,j]*1000, label=label)
                axes[0,1].plot(x, example['observable_displacement_m'][:,j]*1000, label=label)
            axes[0,0].set_title('Raw per-window antenna displacement')
            axes[0,1].set_title('Same displacement after quadratic removal')
            for ax in axes[0]:
                ax.set_ylabel('mm');ax.legend(ncol=3,fontsize=8)
            for j,s in enumerate(carrier.signals):
                axes[1,0].plot(x, example['phase_residual_cycles'][:,j], '.-', label=s)
            axes[1,0].set(title='Observed carrier residuals',ylabel='cycles')
            axes[1,0].legend(ncol=3,fontsize=7)
            values = [np.linalg.norm(row)/meta['wavelength_m'] for row in example['observable_displacement_m']]
            axes[1,1].plot(x, values, color='#255fa0')
            axes[1,1].set(title='Largest possible LOS projection magnitude |d|/λ',ylabel='cycles')
            for ax in axes.flat:ax.set_xlabel('Seconds within selected window')
            fig.suptitle(f"Example window {example['index']} · no satellite LOS fit or spoofing decision")
            save(fig, 'example_window')
    return names


def write_report(output, meta, carrier, imu, records, profile, plots=True):
    output = Path(output)
    names = make_plots(output, meta, carrier, imu, records, profile) if plots else []
    images = {}
    for path in names:
        encoded = base64.b64encode((output/path).read_bytes()).decode('ascii')
        images[path.stem] = f'<img loading="lazy" src="data:image/png;base64,{encoded}" alt="{escape(path.stem.replace("_"," "))}">'
    counts = meta['integration_status_counts']
    integrated = counts.get('integrated', 0)
    noise_rows = []
    for sigma in meta['noise_sensitivity_sigmas_m_s2']:
        checks = [c for r in records for c in r['noise_sensitivity'] if c['sigma_m_s2']==sigma]
        checked = [c for c in checks if c.get('applied')]
        noise_rows.append(f'<tr><td>{sigma:g}</td><td>{len(checked)}</td><td>{sum(c["passed"] for c in checked)}</td><td>{sum(not c["passed"] for c in checked)}</td></tr>')
    px4 = meta['px4']
    geometry = meta.get('geometry')
    geometry_text = 'Not available; receiver origin and navigation ephemerides have not both been supplied.'
    if geometry:
        origin=geometry['origin']
        geometry_text=(f"Fixed origin: {origin['latitude_deg']:.7f}° N, {origin['longitude_deg']:.7f}° E, "
                       f"{origin['ellipsoid_height_m']:.3f} m ellipsoidal height. The user authorized GNSS position "
                       'from the ULog only for this location. Orbit propagation uses public NOAA GPS LNAV navigation data; '
                       'each window uses its own midpoint epoch. Healthy broadcast records are required within two hours of toe, '
                       'with transmission time no later than the measurement epoch. Transmit-time and Earth-rotation corrections are included.')
    first = imu.time_s[0]
    # Downsample only the long report overview, not exported data/integration.
    stride = max(1, len(imu.time_s)//2500)
    payload = {'meta':meta, 'windows':records, 'signals':carrier.signals,
               'clock_profile':profile, 'imu':{'boot_s':imu.time_s[::stride],
                                               'specific_force':imu.specific_force_body_m_s2[::stride],
                                               'linear_acceleration':imu.linear_acceleration_enu_m_s2[::stride]}}
    data = json.dumps(clean_json(payload),separators=(',',':'),allow_nan=False).replace('<','\\u003c')
    table_rows=[]
    for r in records:
        rms=r.get('observed_total_rms_m')
        cells=[r['index'],f"{r['start_s']:.0f}–{r['end_s']:.0f}",r['samples'],r['common_signals'],
               'yes' if r['availability_pass'] else 'no',r['integration_status'],
               '—' if rms is None else f'{rms*1000:.3f}']
        table_rows.append('<tr>'+''.join('<td>'+escape(str(c))+'</td>' for c in cells)+'</tr>')
    lever = px4['lever_arm_body_m']
    lever_text = 'Unknown: plotted displacement is IMU only.' if lever is None else f'FRD offset: [{lever[0]:g}, {lever[1]:g}, {lever[2]:g}] m, supplied by the user.'
    overview = (f'{len(carrier.tow):,} raw epochs · {len(carrier.signals)} GPS L5_Q signals · '
                f'{meta["phase_duration_s"]:.2f} seconds of receiver epochs · {len(imu.time_s):,} usable IMU samples. '
                f'{integrated}/{len(records)} windows integrated under the selected provisional clock. '
                f'{meta["availability_pass_windows"]} pass basic phase sample/gap availability. '
                'Availability and integration success do not establish detector readiness.')
    replacements={
        '__DATA__':data, '__SUMMARY__':escape(overview), '__EPOCHS__':str(len(carrier.tow)),
        '__SIGNALS__':str(len(carrier.signals)), '__WINDOWS__':str(integrated),
        '__DELAY__':f'{meta["clock"]["receive_delay_growth_s"]:.2f}',
        '__CLOCK_PLOT__':images.get('clock_alignment',''), '__IMU_PLOT__':images.get('px4_motion',''),
        '__PHASE_PLOT__':images.get('carrier_quality',''), '__QUALITY_PLOT__':images.get('motion_quality',''),
        '__EXAMPLE_PLOT__':images.get('example_window',''), '__TABLE__':''.join(table_rows),
        '__NOISE_TABLE__':''.join(noise_rows), '__LEVER__':escape(lever_text),
        '__ANCHOR__':f'{meta["clock"]["anchor_boot_s"]:.9f}',
        '__OFFSET__':f'{meta["selected_offset_s"]:g}',
        '__NOISE_PROXY__':escape(str(clean_json(px4['ground_noise_proxy_axis_m_s2']))),
        '__RESETS__':escape(', '.join(f'{r:.6f}' for r in px4['attitude_reset_boot_s'])),
        '__EXPECTED__':escape(meta['labels']['expected_case']),
        '__DISCOVERY__':str(profile['best_discovery_offset_s']),
        '__HOLDOUT__':str(profile['best_holdout_offset_s']),
        '__BLOCKERS__':''.join('<li>'+escape(b)+'</li>' for b in meta['blockers']),
        '__GEOMETRY_TEXT__':escape(geometry_text), '__SKY_PLOT__':images.get('satellite_sky',''),
        '__CLOCK_STATUS__':escape(meta['offset_status']),
        '__SCREEN_COUNTS__':escape(str(meta['conditional_quality_screen'])),
        '__GEOMETRY_LINKS__':('<a href="satellite_geometry.csv">Satellite directions CSV ↗</a> · <a href="satellite_geometry.json">Geometry provenance ↗</a>' if geometry else ''),
        '__PDF_LINK__':'<a href="flight_inspection.pdf">All figures as PDF ↗</a>' if names else '',
    }
    document = Path(__file__).with_name('recorded_flight_report_template.html').read_text()
    for marker,value in replacements.items():
        document=document.replace(marker,value)
    (output/'report.html').write_text(document)
