# SmartKnob speed limit validation

Date: 2026-10-22 (Asia/Shanghai). The user confirmed that the earlier coarse
oscillation was gone and requested a higher speed before haptics are disabled.
The connected board was confirmed in Coarse Strong (preset 8) before testing.

## Current limits: additional 50 percent

The subsequent user request increases the 120 rad/s version by another 50%:

| Limit | Previous | Current |
| --- | --- | --- |
| Haptic speed cutoff | 120 rad/s | 180 rad/s (about 1719 rpm) |
| Recovery after cutoff | 80 rad/s | 120 rad/s (about 1146 rpm) |
| Raw encoder sample speed envelope | 160 rad/s | 240 rad/s |

At 170 rad/s a 1-degree fine preset crosses about 9.74 detents per 1 ms step.
The transition budget therefore increases from 8 to 16 per step, retaining a
bounded loop while preventing fine detents from falling behind. Preset force
and current limits are unchanged. Diagnostic abort speed remains 200 rad/s.

### Additional increase verification

All 33 CTest tests passed. Coarse integrated-motion checks retain +/-70/110
rad/s and add +/-130/170 rad/s. Boundary tests cover 179 -> 180 -> 150 ->
121 -> 120 -> 150 rad/s, a valid 239 rad/s raw step, rejection at 250 rad/s,
and two-sample recovery. The fast-release model matrix now has 96 cases.
Fine Smooth/Detents at +/-130/170 rad/s failed with the old 8-transition
budget (residual 1.286668 detents exceeded snap 1.1), then passed with 16.
These are software tests, not physical speed measurements.

Seven Coarse Strong hardware captures completed normally using bounded
+/-250 mA pulses. Five normal captures use 20 ms sampling over 2500 ms:

| Pulse | Sampled post-release peak magnitude | Settling | Late angle span |
| --- | --- | --- | --- |
| -250 mA, 125 ms | 140.99 rad/s | 325 ms | 0.044 degrees |
| -250 mA, 150 ms | 161.07 rad/s | 380 ms | 0.044 degrees |
| -250 mA, 170 ms | 167.73 rad/s | 420 ms | 0.110 degrees |
| -250 mA, 190 ms | 170.88 rad/s | 400 ms | 2.175 degrees |
| +250 mA, 150 ms | 159.39 rad/s | 420 ms | 1.758 degrees |

The same settling and late-window definitions below apply. Each normal
record shows one above-threshold velocity reversal, consistent with bounded
endstop return; these results do not imply zero rebound or zero slow drift.
The 170.88 rad/s release sample retained a +450 mA braking command. Physical
tests did not reach 180 rad/s, so cutoff/recovery boundary verification for
this revision is from software tests only. Diagnostic abort remains 200 rad/s.

Two additional +/-250 mA, 150 ms pulse captures record 466..849 ms at 1 kHz.
Their post-release sampled peaks are 161.56/162.64 rad/s. These short windows
do not independently confirm settling. All seven records report zero
cumulative control deadline misses. Raw captures, logs and summary are in
`build/coarse-speed180`; diagnostic ELF is in its `firmware` subdirectory.
The pre-change Flash backup is `build/pre-speed180-20261022-flash.bin`.

The final normal Release image (diagnostics OFF) was flashed, verified and
reset: `build/Release/ROLLERCAN_standalone.hex`. SHA-256:
`D4D90202B7166D7CCFFA992B2A157157E0DF581658825A1E40EB54FBE397DB05`.
Flash usage is 99,168 bytes and RAM usage is 24,488 bytes. Live reads confirm
preset 8, Dial mode, enabled output, no error/overvoltage, zero control
deadline misses and zero encoder DMA error/overlap/stale/unexpected IRQ
counts. No diagnostic preset-save command was issued.

The following sections retain the earlier 120 rad/s validation history.

## Initial limit increase

| Limit | Before | After |
| --- | --- | --- |
| Haptic speed cutoff | 60 rad/s | 120 rad/s (about 1146 rpm) |
| Recovery after cutoff | 40 rad/s | 80 rad/s (about 764 rpm) |
| Raw encoder sample speed envelope | 80 rad/s | 160 rad/s |

These limits apply to ordinary SmartKnob modes and navigation. FR-L still
bypasses the speed latch and uses its velocity-consistency sample check.
The sample envelope was increased together with the cutoff so valid motion
at 80..120 rad/s is not silently rejected by the position validator.
Hysteresis, current ceilings, the 8 ms velocity estimator and damping gains
remain in use. Single-center accelerating-current derating remains 20..45
rad/s; that is separate from the coarse speed cutoff.

## Verification

The pre-change Coarse Strong retest used the earlier diagnostic ELF in
`build/coarse-fixed`: +/-25 rad/s targets with 200 ms drive pulses.
Settling was 90 ms into detents and 250 ms from outside the endstop.

The updated diagnostic ELF is in `build/coarse-speed-limit/firmware`.
Short current pulses were increased from -150 to -250 mA for 80 ms,
followed by -250 mA for 100/125 ms. All captures completed normally.
Normal records cover 2500 ms at 20 ms intervals. Selected bidirectional
100 ms pulses were also captured at 1 kHz over 416..799 ms.

Observed Coarse Strong behavior:

- -83.78 rad/s after release: +450 mA braking command, 130 ms settling.
- -105.90 rad/s after release: +450 mA braking command, 290 ms settling.
- -139.96 rad/s after release: zero commanded current. At -78.64 rad/s,
  +450 mA resumed. This run settled 345 ms after release.
- Dense bidirectional records reached sampled speed magnitudes of 117.81
  and 115.28 rad/s. They showed a bounded rebound, with final sampled speeds
  of +0.074 and -0.034 rad/s at 349 ms after release.
- Every run reported zero cumulative control deadline misses. The three
  normal high-speed runs above had late angle spans of 0.022, 0.110 and
  0.066 degrees respectively, measured from 500 ms after release onward.

Settling requires all subsequent recorded speeds below 0.5 rad/s with at
least 200 ms confirmation. Values are sampled measurements; 20 ms records
cannot resolve between-sample peaks. The short dense records do not have a
long enough final confirmation window to independently certify settling.

All 32 CTest tests passed. New tests exercise integrated position at +/-70
and +/-110 rad/s, exact 120/80 hysteresis, independent raw-sample rejection
above 160 rad/s, and two-sample recovery. An additional 48 closed-loop model
scenarios cover strong/weak modes, both directions, endpoints and friction
levels. These are software models, distinct from the hardware measurements.

Raw data and logs are under `build/coarse-speed-limit`. The pre-change Flash
backup is `build/pre-speed-limit-20261022-flash.bin`. No diagnostic preset-save
command was issued. The optional diagnostic now also accepts dense capture
of bounded current pulses, keeping its existing 250 mA and 200 rad/s limits.

Example reproduction after flashing the diagnostic image:

```powershell
node tools/motor_diagnostics.mjs --serial 01290141524300143638414B --elf build/coarse-speed-limit/firmware/ROLLERCAN.elf --out build/coarse-speed-limit/strong-over-limit.json --mode 8 --current-ma -250 --pulse-ms 125 --duration-ms 2500
node tools/summarize_coarse_release.mjs build/coarse-speed-limit/strong-over-limit.json
```

## Initial 120 rad/s firmware

The normal Release image (diagnostics OFF) was flashed, verified and reset:
`build/Release/ROLLERCAN_standalone.hex`. SHA-256:
`CB927ECFB6E15BD4E7D62E1DFC8CE5449C24C1AFFF4F93DDF0A28C275EEF3F72`.
Flash usage is 99,168 bytes and RAM usage is 24,488 bytes.
Live reads after reset confirmed preset 8 (Coarse Strong), Dial mode,
enabled output, no error/overvoltage, zero control deadline misses, and zero
encoder DMA error/overlap/stale/unexpected IRQ counts.
