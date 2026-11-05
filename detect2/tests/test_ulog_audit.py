import numpy as np

from SpoofingDetection.detect2.acquisition.ulog_audit import topic_summary


def test_sample_timing_exposes_duplicates_backwards_gaps_and_reset_changes():
    timestamps=np.array([10000,15000,20000,20000,19000,50000,55000])
    data={'timestamp':timestamps,'quat_reset_counter':np.array([0,0,0,1,1,1,1]),
          'accel_calibration_count':np.zeros(7,dtype=int),'accel_device_id':np.full(7,1234)}
    result=topic_summary(data)
    assert result['samples']==7 and result['distinct_timestamps']==6
    assert result['duplicate_timestamps']==1 and result['backwards_timestamps']==1
    assert result['large_gaps']==1 and result['max_interval_us']==31000
    assert result['provenance_and_reset_fields']['quat_reset_counter']['changes']==1
    assert result['provenance_and_reset_fields']['accel_device_id']['first']==1234


def test_realized_rate_uses_distinct_source_times_not_requested_rate():
    result=topic_summary({'timestamp':np.arange(10000,1010000,5000,dtype=np.int64)})
    assert result['average_hz']==200 and result['large_gaps']==0
    assert result['duplicate_timestamps']==0 and result['backwards_timestamps']==0


def test_supplied_sample_times_distinguish_publication_jitter_from_sample_gaps():
    result=topic_summary({'timestamp':np.array([10000,15000,23000,25000]),
                          'timestamp_sample':np.array([10000,15000,20000,25000])})
    assert result['timestamp_source']=='timestamp_sample'
    assert result['large_gaps']==0 and result['average_hz']==200
    assert result['publication_timing']['large_gaps']==1
    assert result['publication_timing']['max_interval_us']==8000
