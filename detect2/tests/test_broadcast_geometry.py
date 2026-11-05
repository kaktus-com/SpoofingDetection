import numpy as np
from numpy.testing import assert_allclose
import pytest

from SpoofingDetection.detect2.broadcast_geometry import (
    EARTH_RATE, MU, nominal_geometry, read_gps_navigation, satellite_ecef, week_difference,
)
from SpoofingDetection.detect2.geometry import LocalFrame


def circular():
    record={k:0. for k in ['crs','delta_n','m0','cuc','eccentricity','cus','toe','cic','omega0',
                          'cis','i0','crc','argument_perigee','omega_dot','i_dot']}
    record.update(prn=3,sqrt_a=np.sqrt(26560000.),week=2426,health=0.,transmission_time=0.)
    return record


def test_broadcast_orbit_matches_analytic_circular_equatorial_ecef():
    record=circular();t=np.array([0.,50.,500.,1200.])
    a=record['sqrt_a']**2;rate=np.sqrt(MU/a**3)-EARTH_RATE
    expected=a*np.column_stack([np.cos(rate*t),np.sin(rate*t),np.zeros(len(t))])
    assert_allclose(satellite_ecef(record,t),expected,atol=1e-8)
    assert_allclose(satellite_ecef(record,604800.+50),expected[1],atol=1e-8)


def test_nominal_geometry_rejects_wrong_week_unhealthy_stale_or_not_yet_transmitted():
    record=circular()
    good=nominal_geometry([record],['0:3:7'],100,LocalFrame(0,0,0),gps_week=2426)[0]
    assert_allclose(sum(good[k]**2 for k in ['nominal_los_enu_east','nominal_los_enu_north','nominal_los_enu_up']),1,atol=1e-15)
    for modified in [{**record,'week':2425},{**record,'health':1},{**record,'toe':10000},{**record,'transmission_time':200}]:
        with pytest.raises(ValueError,match='No healthy recent'):
            nominal_geometry([modified],['0:3:7'],100,LocalFrame(0,0,0),gps_week=2426)


def test_rinex2_fixed_width_negative_and_d_exponent_parser(tmp_path):
    values=[32,-20,.0000000044,1.46,-.0000012,.0018,.0000088,5153.6,
            165600,.00000003,.88,.00000003,.957,205,.195,-.0000000081,
            -.000000000159,1,2426,0,2,0,-.0000000088,544,160410,4,0,0]
    lines=['     2.11           N: GPS NAV DATA                         RINEX VERSION / TYPE',
           '                                                            END OF HEADER',
           ' 1 26  7  6 22  0  0.0']
    lines += ['   '+''.join(f'{v:19.12E}'.replace('E','D') for v in values[i:i+4]) for i in range(0,28,4)]
    p=tmp_path/'nav.26n';p.write_text('\n'.join(lines)+'\n')
    r=read_gps_navigation(p)
    assert len(r)==1 and r[0]['prn']==1 and r[0]['week']==2426
    assert r[0]['crs']==-20 and r[0]['eccentricity']==pytest.approx(.0018)
    assert r[0]['transmission_time']==160410


def test_week_rollover_difference_has_correct_sign():
    assert_allclose(week_difference([604799,1,604801,-604799]),[-1,1,1,1])
