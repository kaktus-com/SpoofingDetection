# Recording a flight

## 1. Power up and connect

Power the Cube, Pi and external X20. Secure their USB/power connections and keep
the drone **disarmed**. Complete your normal aircraft preflight separately;
`RECORDING READY` confirms the recording streams, not aircraft flight readiness.

Connect from your computer and open a persistent terminal session:

```bash
ssh nss@10.15.4.141
tmux new -A -s detect2 -n recorder
cd /home/nss/SpoofingDetection
```

`tmux` keeps the recorder running if SSH/Wi-Fi disconnects. If this reattaches
an existing recording, keep using it rather than starting a second recorder.

## 2. Check the router

```bash
pgrep -a -x mavlink-routerd
```

There should be **one** router. Keep its terminal/process running. Only if none
is listed, start it in a separate tmux window:

```bash
tmux new-window -d -n router '/usr/bin/mavlink-routerd -e 127.0.0.1:14551 /dev/serial/by-id/usb-CubePilot_CubeOrange_0-if00:1500000'
pgrep -a -x mavlink-routerd
```

## 3. Start recording, then fly

```bash
./start_recording --timesync
```

Use this untimed command for flight. Keep the old `start`/`spoofing_detector.py`
recorder stopped.

Wait for **`RECORDING READY`**. The launcher configures the X20 at 20 Hz/2 Mbaud
and starts native PX4 logging with the 128 KiB buffer. Keep the drone still for
about **30 seconds**, then arm and take off using your usual RC/QGroundControl
procedure. The launcher does not arm or control the aircraft.

Leave recording running for the whole flight. To detach from tmux, press
**Ctrl+B**, release, then **D**. Reconnect with:

```bash
ssh nss@10.15.4.141
tmux attach -t detect2
```

## 4. Land and finish

Land, **disarm**, and leave the drone still for about **30 seconds**. In the
recorder terminal, press **Ctrl+C once**. Wait for native-log download and the
final status before powering anything off. `Runtime: stopped` is the normal
result of an intentional Ctrl+C stop.

If it reports `pending_native`, collect after disarm using the printed path:

```bash
./start_recording --finalize /absolute/session/path
```

Recording interruptions remain marked failed. Troubleshoot and start a new
recording on the ground, disarmed. Run the launcher again before each new flight.

## 5. Find the data

The printed session folder is under:

```text
/home/nss/SpoofingDetection/recordings_v2/
```

Keep the **whole folder**. Open `recording_report.html`; it links the raw GNSS,
MAVLink, native `.ulg`, settings and status files. `runtime_manifest.json`
contains the combined result. Clock synchronization is still a separate pending
validation for GNSS epochs. With `--timesync`, open `timesync/report.html` for
the Pi/PX4 clock exchanges and estimated mapping. It does not establish the
GNSS-to-IMU timing bridge or a spoofing verdict. Omitting `--timesync` records
passively, as in the earlier flights.

See [the detailed runtime guide](acquisition/RUNTIME.md) for failure handling.
