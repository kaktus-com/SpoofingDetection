# CubeOrange PPS input: research and proposed next check

Research date: 5 November 2026. **Yes: this CubeOrange firmware can timestamp
PPS edges through an FMU AUX signal pin.** An additional timestamping board is
not presently indicated. Physical wiring and pulse capture have not been tested.

The read-only inspection is saved in
[cube_readonly_20261105.json](research/pps_input/cube_readonly_20261105.json).
The repeatable [inspection script](research/pps_input/inspect_cube.py) leaves
the production recorder's command restrictions unchanged.

## Why the connector says AUX OUT

The label describes its usual servo/PWM role. PX4 explicitly assigns input
functions, including PPS and camera capture, to FMU output pins in its
[actuator documentation](https://docs.px4.io/main/en/config/actuators#output-functions).
The same documentation identifies AUX as the FMU bus. For PPS the pin is
reconfigured as an input, rather than driven as an output at the same time.
The matched CubeOrange board source even designates AUX5 as the PWM-input pin.
This supports configurable input use; it does not independently verify every
third-party carrier's buffering or connector layout. The exact carrier signal
path and a real input-edge test remain manual checks before accepting AUX6.

## Confirmed on this aircraft

The controller identifies as `CUBEPILOT_CUBEORANGE`, UID
`3544703520294189106`, running PX4 commit
`cb411db5b54a98442ab974292c81b74c057bdf25` (`more-configts`).

| Item | Read-only result |
|---|---|
| `pps_capture status` | Command exists; driver not running |
| `PPS_CAP_ENABLE` | 0 |
| `PPS_CAP_GPS_ID` | 0 |
| `PWM_AUX_FUNC1` … `PWM_AUX_FUNC6` | All 0 / unassigned |
| `PWM_AUX_TIM0`, `PWM_AUX_TIM1` | Both 400 / PWM enabled |
| MAIN 1–4 | Motor functions 101–104 |
| Camera capture / PWM input drivers | Both stopped |
| Lidar PWM / external ATS enables | Both 0 |

The source at the same commit enables `CONFIG_DRIVERS_PPS_CAPTURE`. The driver
finds function **2064 (PPS Input)** on `PWM_AUX_FUNCn` for this IO-equipped board.
It timestamps the rising-edge GPIO interrupt with `hrt_absolute_time()` and
publishes `pps_capture.timestamp` in microseconds on the PX4 boot clock. The
native logger already lists `pps_capture` as an optional logged topic.
This establishes software support; interrupt latency remains to be measured.
It is not a demonstrated hardware-latched nanosecond timestamp.

The checked source files and their hashes are in
[source_fingerprints.json](research/pps_input/source_fingerprints.json).
PX4 also documents the [PPS driver](https://docs.px4.io/main/en/modules/modules_driver#pps_capture)
and [PPS input function](https://docs.px4.io/main/en/config/actuators#output-functions).
The generic actuator page currently has an inconsistent enable-value sentence;
the installed firmware's startup code starts the driver when
`PPS_CAP_ENABLE > 0`, and its parameter metadata says enabled = 1.

## Candidate: AUX6, subject to the physical check

The matched board mapping is:

| Carrier AUX label | MCU pin | Timer bank |
|---|---|---|
| AUX1–4 | PE14, PE13, PE11, PE9 | TIM1 / `PWM_AUX_TIM0` |
| AUX5 | PD13 | TIM4 / `PWM_AUX_TIM1` |
| AUX6 | PD14 | TIM4 / `PWM_AUX_TIM1` |

**AUX6 is a proposed choice**, inferred from these mappings and the unused AUX
functions. AUX5 is also the board's designated PWM-input pin, so AUX6 avoids
reusing that signal. AUX5 and AUX6 still share a timer bank.
Unassigned functions alone do not release that bank: `pwm_out status` confirms
both channels are currently owned for PWM. A future reviewed configuration would
reserve AUX5/6 for input use, approximately:

| Parameter | Current | Proposed, not applied |
|---|---:|---:|
| `PWM_AUX_TIM1` | 400 | 0 / release PWM on AUX5 and AUX6 |
| `PWM_AUX_FUNC6` | 0 | 2064 / PPS Input |
| `PPS_CAP_ENABLE` | 0 | 1 / enable capture at startup |

This is a research proposal, **not an instruction to change these values now**.
Timer allocation and driver startup need an explicit disarmed bench check after
the normal required reboot. Keep MAIN motor assignments unchanged.

You confirmed a **3.3 V PPS output**. The remaining physical check is the exact
carrier model, the AUX6 signal/ground labels, and whether AUX5/6 have anything
connected. Carrier layouts vary; the supplied board name did not identify one
unambiguously. The proposed connections are PPS to the verified AUX6 **signal**
contact and a common ground. The servo power rail is a separate contact, not the
PPS input. See the manufacturer's
[interface specification](https://github.com/CubePilot/cubepilot-docs/blob/master/autopilot/the-cube/introduction/interface-specifications.md).

## Associate edges with the external X20, not the onboard GPS

`pps_capture.timestamp` is the useful PX4 edge timestamp. The driver's
`rtc_timestamp` is derived from a PX4 `sensor_gps` instance and a next-second
assumption. Because the X20 currently connects to the Pi, that GNSS instance
does **not** identify the external receiver's pulse. Do not treat this field as
validated X20 time, and do not align from onboard GNSS motion or detector output.

Instead, record the X20's pulse-time metadata and RAWX week/TOW, verify the pulse
time grid, polarity and validity, and pair pulse epochs with PX4 edge timestamps.
Handle missing pulses, reset segments and leap-second/timebase conversion
explicitly. The u-blox
[X20 integration manual](https://content.u-blox.com/sites/default/files/documents/ZED-X20P_IntegrationManual_UBXDOC-963802114-12901.pdf)
describes `UBX-TIM-TP` as labeling an upcoming pulse. Its recommendation for a
simple 1 Hz association does not validate our existing 20 Hz measurement setup.
The current online manual is R05 and targets newer firmware than our HPG 2.00 /
protocol 50.02; exact configuration/message support must be checked before any
receiver changes. Preserve the working 20 Hz RAWX profile during that review.

## Manual checkpoints before deployment

1. Verify the carrier and unused AUX5/6 contacts; review the proposed timer-bank
   change and export the existing parameters.
2. With the aircraft disarmed, configure the reviewed input, connect the verified
   signal/ground and check rising edges at 1 Hz. Confirm `pps_capture` really
   publishes, its rate-exceeded counter stays clear, and the native ULog stores it.
3. Confirm which X20 time message labels each pulse while RAWX stays at 20 Hz.
   Plot missing/duplicate edges, offset and drift in a three-minute ground test.
4. Independently check latency/uncertainty against the detector's timing budget.
   TIMESYNC residuals alone do not establish this error budget.

No driver starts, parameter writes, reboot or wiring changes were made for this
research. Existing flight recordings remain unchanged.
