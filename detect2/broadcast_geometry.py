"""GPS LNAV broadcast orbit → nominal ENU line of sight, from first principles.

IS-GPS-200N, Table 20-IV. Only GPS RINEX 2 navigation input is supported.
Satellite clock corrections are unnecessary for this approximate LOS geometry;
transmit-time range and Earth rotation during propagation are iterated.
"""

from datetime import datetime, timedelta
import gzip
from pathlib import Path

import numpy as np

from .geometry import LocalFrame

MU = 3.986005e14
EARTH_RATE = 7.2921151467e-5
SPEED_OF_LIGHT = 299792458.
GPS_EPOCH = datetime(1980, 1, 6)


def week_difference(seconds):
    return (np.asarray(seconds)+302400.) % 604800.-302400.


def read_gps_navigation(path):
    path = Path(path)
    text = gzip.decompress(path.read_bytes()).decode('ascii') if path.suffix == '.gz' else path.read_text()
    lines = text.splitlines()
    if not lines or not lines[0].lstrip().startswith('2.'):
        raise ValueError('Expected a RINEX 2 GPS navigation file')
    start = next((i+1 for i, line in enumerate(lines) if 'END OF HEADER' in line), None)
    if start is None:
        raise ValueError('Navigation header is incomplete')
    fields = ('iode','crs','delta_n','m0','cuc','eccentricity','cus','sqrt_a',
              'toe','cic','omega0','cis','i0','crc','argument_perigee','omega_dot',
              'i_dot','codes_l2','week','l2_flag','ura','health','tgd','iodc',
              'transmission_time','fit_hours','spare1','spare2')
    output = []
    for offset in range(start, len(lines), 8):
        block = lines[offset:offset+8]
        if len(block) != 8:
            raise ValueError('Truncated navigation record')
        first = block[0]
        epoch = first[:22].split()
        year = int(epoch[1])+2000 if int(epoch[1]) < 80 else int(epoch[1])+1900
        dt = datetime(year, int(epoch[2]), int(epoch[3]), int(epoch[4]), int(epoch[5]))+timedelta(seconds=float(epoch[6]))
        values = []
        for line in block[1:]:
            padded = line.ljust(79)
            values.extend(float(padded[3+19*j:3+19*(j+1)].replace('D','E').strip() or 'nan') for j in range(4))
        record = dict(zip(fields, values))
        record.update(prn=int(epoch[0]), toc_gps_seconds=(dt-GPS_EPOCH).total_seconds())
        essential = [record[k] for k in fields[:17]]+[record['week'],record['health']]
        if not np.isfinite(essential).all() or not 0 <= record['eccentricity'] < 1 or record['sqrt_a'] <= 0:
            raise ValueError('Invalid GPS orbit parameters')
        output.append(record)
    return output


def satellite_ecef(record, tow_s):
    a = record['sqrt_a']**2
    tk = week_difference(np.asarray(tow_s)-record['toe'])
    mean = record['m0']+(np.sqrt(MU/a**3)+record['delta_n'])*tk
    eccentricity = record['eccentricity']
    anomaly = np.array(mean, copy=True)
    for _ in range(20):
        step = (anomaly-eccentricity*np.sin(anomaly)-mean)/(1-eccentricity*np.cos(anomaly))
        anomaly -= step
        if np.max(np.abs(step)) < 1e-13:
            break
    else:
        raise ValueError('Kepler orbit did not converge')
    true = np.arctan2(np.sqrt(1-eccentricity**2)*np.sin(anomaly),np.cos(anomaly)-eccentricity)
    phi = true+record['argument_perigee']
    ss, cc = np.sin(2*phi),np.cos(2*phi)
    u = phi+record['cus']*ss+record['cuc']*cc
    radius = a*(1-eccentricity*np.cos(anomaly))+record['crs']*ss+record['crc']*cc
    inclination = record['i0']+record['i_dot']*tk+record['cis']*ss+record['cic']*cc
    longitude = record['omega0']+(record['omega_dot']-EARTH_RATE)*tk-EARTH_RATE*record['toe']
    xp,yp = radius*np.cos(u),radius*np.sin(u)
    return np.stack([xp*np.cos(longitude)-yp*np.cos(inclination)*np.sin(longitude),
                     xp*np.sin(longitude)+yp*np.cos(inclination)*np.cos(longitude),
                     yp*np.sin(inclination)],axis=-1)


def nominal_geometry(records, signals, tow_s, frame: LocalFrame, max_age_s=7200., gps_week=None):
    rows = []
    receiver = frame.origin_ecef_m
    for signal in signals:
        gnss, prn, sig = map(int, signal.split(':'))
        if gnss != 0:
            raise ValueError('GPS broadcast geometry requires GPS signal identities')
        candidates = [r for r in records if r['prn']==prn and r['health']==0
                      and (gps_week is None or int(r['week'])==gps_week)
                      and np.isfinite(r['transmission_time'])
                      and float(week_difference(tow_s-r['transmission_time'])) >= 0
                      and abs(float(week_difference(tow_s-r['toe']))) <= max_age_s]
        if not candidates:
            raise ValueError(f'No healthy recent ephemeris for {signal}')
        record = min(candidates,key=lambda r:abs(float(week_difference(tow_s-r['toe']))))
        transit = .075
        for _ in range(5):
            transmit_ecef = satellite_ecef(record,tow_s-transit)
            angle = EARTH_RATE*transit
            c,s=np.cos(angle),np.sin(angle)
            position = np.array([c*transmit_ecef[0]+s*transmit_ecef[1],
                                 -s*transmit_ecef[0]+c*transmit_ecef[1],transmit_ecef[2]])
            transit = np.linalg.norm(position-receiver)/SPEED_OF_LIGHT
        difference = position-receiver
        los = frame.ecef_vector_to_enu(difference/np.linalg.norm(difference))
        rows.append({'signal_id':signal,'reference_gps_tow_s':float(tow_s),'gps_week':int(record['week']),
                     'nominal_los_enu_east':float(los[0]),'nominal_los_enu_north':float(los[1]),'nominal_los_enu_up':float(los[2]),
                     'azimuth_deg':float(np.degrees(np.arctan2(los[0],los[1]))%360),
                     'elevation_deg':float(np.degrees(np.arcsin(los[2]))),
                     'range_m':float(np.linalg.norm(difference)),'ephemeris_age_s':float(week_difference(tow_s-record['toe'])),
                     'ephemeris_healthy':True,'toe_s':float(record['toe'])})
    return rows


def origin_from_ulog(path):
    """Explicitly authorized exception: GNSS location only, never displacement."""
    from pyulog import ULog
    log = ULog(str(path),message_name_filter_list=['vehicle_gps_position'])
    data = next((ds.data for ds in log.data_list if ds.name=='vehicle_gps_position'),None)
    if data is None:
        raise ValueError('No vehicle_gps_position for the authorized location lookup')
    ok = np.asarray(data['fix_type']) >= 3
    positions=np.column_stack([data[k] for k in ['latitude_deg','longitude_deg','altitude_ellipsoid_m']])
    ok &= np.isfinite(positions).all(axis=1)
    if not np.any(ok):
        raise ValueError('No finite 3D GNSS fixes for approximate location')
    return LocalFrame(*np.median(positions[ok],axis=0))
