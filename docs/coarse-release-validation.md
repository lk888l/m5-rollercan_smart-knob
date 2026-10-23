# Coarse release stability validation

This records the initial damping fix. The subsequent user-requested speed
limit increase and final firmware are recorded in
[speed limit validation](smartknob-speed-limit-validation.md).

Date: 2026-10-22 (Asia/Shanghai). Hardware: the user's RollerCAN motor,
ST-Link `01290141524300143638414B`. The original 128 KiB Flash was backed up to
`build/pre-smartknob-20261022-flash.bin` before flashing.

## Cause and implementation

Coarse previously used `motor_rps`, the legacy 2 Hz speed filter with a time
constant of about 80 ms. During a quick reversal its sign can disagree with
the shaft motion. The `-D * velocity` term then adds energy to the rebound.
Existing tests supplied identical fast and slow velocities and missed this.

All SmartKnob modes, navigation, idle detection, speed protection and haptic
telemetry now use the 8 ms `motor_rps_fast` estimate. Preset widths, stiffness,
clicks, current limits, CAN parameter IDs and persistence layout are retained.

The motion estimator was extracted from `mysys.c`, following the reference
project's `Application/include/roller/control/math.hpp` AngleTracker pattern:
integer turns plus full 14-bit absolute counts, position and velocity from
the same sample, and measured elapsed time. The HAL-independent module handles
initialization, calibration reset, wrapping, duplicate timestamps and long
sampling gaps. Legacy speed PID retains its existing 2 Hz estimate. The
reference project's ADC/FOC timing changes were not transplanted as part of
this focused motion-feedback refactor.

The diagnostic now observes all presets during the simulated hand motion,
so detent transitions and position filters are current at release. Both
compared firmware versions use this identical diagnostic change.

## Hardware procedure

- Baseline: revision `40a15d6`, isolated at `build/coarse-baseline-src`, with
  only the diagnostic observer condition changed. Built at
  `build/coarse-baseline-fair`.
- Fixed diagnostic firmware: `build/coarse-fixed`.
- Both use Release optimization, 1 kHz ControlTask, preset 8/9 defaults and
  a 450 mA haptic current limit.
- A 200 ms velocity-controlled pulse ramps toward +/-25 rad/s, with drive
  current limited to 150 mA. Release occurs 550 ms after test start. Tests
  automatically turn off the bridge at their bounded end time.
- Starting preset position is 0: negative motion traverses detents; positive
  motion stretches the lower endstop. Absolute rotor angle varies between
  runs, so these are matched test conditions, not identical trajectories.
- Ordinary captures cover 2500 ms at 20 ms intervals. Dense captures cover
  516..899 ms at 1 ms intervals, including release and its first 349 ms.
- Every completed run reported status 2 and zero cumulative control deadline
  misses. The first exploratory files under `build/coarse-baseline` use the
  old observer behavior and are excluded from the comparisons below.

## Measurements

Settling means all later recorded speeds stay below 0.5 rad/s, with at least
200 ms of confirmation remaining. Values below come from the 20 ms records;
they are sample-limited measurements, not between-sample maxima or guarantees
for every manual gesture.

| Scenario | Baseline settling | Fixed settling |
| --- | --- | --- |
| Strong, into detents, repeat run | 430 ms | 70 ms |
| Strong, into detents, first run | Not confirmed in capture | 90 ms |
| Strong, outside endstop | 390 ms | 210 ms |
| Weak, into detents | 390 ms | 90 ms |
| Weak, outside endstop | 410 ms | 290 ms |

The baseline first strong run only crossed below the threshold for its final
60 ms, so it does not satisfy the 200 ms confirmation requirement. Its late
angle span was 0.80 degrees versus 0.044 degrees in the corresponding fixed
run. Fixed positive endstop captures had 0.15..0.20 degrees of slow late
movement; this is not a claim of zero motion.

The independent 1 kHz strong inward capture showed 14 significant velocity
reversals before the baseline capture ended, versus five short reversals in
the fixed capture, which settled at 75 ms. At 349 ms after release the sampled
speed was -10.39 rad/s before and -0.021 rad/s after. The fixed weak inward
dense capture settled at 99 ms. Brief spring rebound remains; sustained large
oscillation was suppressed in these tests. Dense endstop captures have too
little remaining time to independently confirm the 200 ms settling window.

Additional fixed-firmware tests covered +/-15 rad/s frictionless operation,
return-to-center, and -40 rad/s coarse drive targets. All completed normally.
FR-L retained free coasting in both directions. These are smoke tests, not a
fresh friction-compensation calibration.

Raw JSON/logs remain under `build/coarse-baseline-fair` and `build/coarse-fixed`.
The generated summary is `build/coarse-release-summary.json`; the plot is
`build/coarse-release-comparison.png`.

## Reproduce and inspect

After flashing the matching diagnostic ELF/standalone HEX:

```powershell
node tools/motor_diagnostics.mjs --serial 01290141524300143638414B --elf build/coarse-fixed/ROLLERCAN.elf --out build/coarse-fixed/strong-negative.json --mode 8 --speed-rad-s -25 --pulse-ms 200 --duration-ms 2500
node tools/motor_diagnostics.mjs --serial 01290141524300143638414B --elf build/coarse-fixed/ROLLERCAN.elf --out build/coarse-fixed/strong-negative-dense.json --mode 8 --speed-rad-s -25 --pulse-ms 200 --duration-ms 900 --dense 1
node tools/summarize_coarse_release.mjs build/coarse-baseline-fair/strong-negative.json build/coarse-fixed/strong-negative.json
python tools/plot_coarse_release.py build/coarse-baseline-fair build/coarse-fixed build/coarse-release-comparison.png
```

Plotting requires matplotlib. The summary tool has no package dependencies
and reports sampling intervals, incomplete settling windows and invalid data.

## Software verification and delivery

All 30 CTest tests passed, including 24 closed-loop coarse scenarios across
both coarse presets, both directions, center/both endpoints and two friction
levels. The low-friction model reproduces a baseline 33.8..35.76 degree late
oscillation; these figures are simulation results, not hardware measurements.
The fixed model satisfies less than one degree of late movement and less
than 0.5 rad/s late peak speed in all 24 scenarios. Existing menu, button,
RGB, host-control, persistence and frictionless tests also pass.

Release and diagnostic firmware builds succeeded. The legacy Keil source
list includes the new motion estimator, but Keil itself was not built.
Pre-existing compiler warnings remain in ADC/display/unused helper code.

The initial normal firmware (diagnostics OFF) was flashed and verified from
`build/Release/ROLLERCAN_standalone.hex`, SHA-256:
`394915C2A099847297F5AF15B394F2B9555DBF72072450CE8D18D4C4F915A0ED`.
It uses 99,168 bytes of Flash and 24,488 bytes of RAM. A live post-reset read
confirmed Dial mode, enabled output, no error/overvoltage, no control deadline
misses, and no encoder DMA error/overlap/stale/unexpected-IRQ counts. The
persisted selected preset is 5 (return-to-center); select Coarse Strong/Weak
on the existing menu for manual feel verification. No preset was saved by
the diagnostic tests.

Original Flash backup SHA-256:
`4EEDC435C0B330F83BF1698F60CEECF0C53461C2FC090895E6E1EF502554F389`.
