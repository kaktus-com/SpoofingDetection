from pymavlink import mavutil
import time

CONNECTION = "udpout:127.0.0.1:14550"
MESSAGE = b"GNSS SPOOFING DETECTED"

mav = mavutil.mavlink_connection(
    CONNECTION,
    source_system=1,
    source_component=mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
)

mav.mav.heartbeat_send(
    mavutil.mavlink.MAV_TYPE_QUADROTOR,
    mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
    0,
    0,
    mavutil.mavlink.MAV_STATE_ACTIVE,
)

time.sleep(1)

mav.mav.statustext_send(
    mavutil.mavlink.MAV_SEVERITY_CRITICAL,
    MESSAGE,
)

print("Sent:", MESSAGE.decode())


def main():
    send_spoofing_message()


if __name__ == "__main__":
    main()


# Synchronization code parked for later:
#
# from collections import defaultdict, deque
#
#
# def sample_time(sample):
#     if "sync_time" in sample:
#         return sample["sync_time"]
#     if "plot_time" in sample:
#         return sample["plot_time"]
#     return sample["time"]
#
#
# def synchronize_windows(
#     imu_samples,
#     carrier_samples,
#     window_seconds=1.0,
#     min_imu_samples=1,
#     min_carrier_samples=1,
# ):
#     imu_buffer = deque()
#     carrier_buffer = deque()
#
#     imu_iter = iter(imu_samples)
#     carrier_iter = iter(carrier_samples)
#
#     next_imu = next(imu_iter, None)
#     next_carrier = next(carrier_iter, None)
#
#     window_start = None
#
#     while next_imu is not None or next_carrier is not None:
#         next_imu_time = sample_time(next_imu) if next_imu is not None else float("inf")
#         next_carrier_time = (
#             sample_time(next_carrier) if next_carrier is not None else float("inf")
#         )
#
#         if window_start is None:
#             window_start = min(next_imu_time, next_carrier_time)
#
#         window_end = window_start + window_seconds
#
#         while next_imu is not None and sample_time(next_imu) < window_end:
#             imu_buffer.append(next_imu)
#             next_imu = next(imu_iter, None)
#
#         while next_carrier is not None and sample_time(next_carrier) < window_end:
#             carrier_buffer.append(next_carrier)
#             next_carrier = next(carrier_iter, None)
#
#         if (
#             len(imu_buffer) >= min_imu_samples
#             and len(carrier_buffer) >= min_carrier_samples
#         ):
#             carrier_by_sat = defaultdict(list)
#
#             for sample in carrier_buffer:
#                 carrier_by_sat[sample["sat"]].append(sample)
#
#             yield {
#                 "start": window_start,
#                 "end": window_end,
#                 "imu": list(imu_buffer),
#                 "carrier": dict(carrier_by_sat),
#             }
#
#         imu_buffer.clear()
#         carrier_buffer.clear()
#         window_start = window_end
