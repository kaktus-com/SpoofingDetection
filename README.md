# SpoofingDetection

## Export synchronized PX4 state

`ulog_state_to_csv.py` exports vehicle-body attitude and fused global position at
the original `vehicle_attitude` rate. Position is interpolated from
`vehicle_global_position`; samples outside its time range, across estimator
resets, across invalid fixes, or across gaps larger than one second are omitted.

```bash
python3 ulog_state_to_csv.py flight.ulg
```

To hold the initial and final state for 30 seconds on either side:

```bash
python3 ulog_state_to_csv.py flight.ulg --prefix-pad 30 --suffix-pad 30
```

Padding uses approximately the original attitude sample rate. Every prefix row
copies the first state and every suffix row copies the last state. Relative time
starts at the beginning of the prefix, so the first recorded sample is at 30
seconds in this example.

The default output is `flight_state.csv` beside the input. It contains
boot-relative sample time, time relative to the first exported row,
roll/pitch/yaw in degrees, WGS-84 latitude/longitude, and AMSL altitude in
metres. Yaw is in `[-180, 180]` using PX4's NED convention.

Dependencies:

```bash
python3 -m pip install numpy pyulog
```
