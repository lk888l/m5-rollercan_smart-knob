// Offline summary of bounded motor_diagnostics hold captures. No hardware I/O.
import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const args = process.argv.slice(2);
const option = name => {
  const index = args.indexOf(name);
  return index < 0 ? undefined : args[index + 1];
};
const directory = option('--dir');
const prefix = option('--prefix') ?? 'final-';
if (!directory) throw new Error('Usage: node tools/summarize_frictionless_hold.mjs --dir <captures> [--prefix final-] [--out report.json]');

// Firmware mean_velocity covers (previous_sample_ms, current_sample_ms].
// Weight partial boundary windows rather than averaging points blindly.
function windowMean(samples, from, to) {
  let integral = 0, duration = 0;
  for (let i = 1; i < samples.length; ++i) {
    const left = Math.max(from, samples[i - 1].ms);
    const right = Math.min(to, samples[i].ms);
    if (right <= left) continue;
    integral += samples[i].mean_velocity_rad_s * (right - left);
    duration += right - left;
  }
  return duration > 0 ? integral / duration : null;
}

const captures = [];
for (const name of readdirSync(directory).filter(name => name.startsWith(prefix) && name.endsWith('.json')).sort()) {
  const capture = JSON.parse(readFileSync(join(directory, name), 'utf8'));
  if (!Array.isArray(capture.samples) || !capture.options || !capture.samples.length) continue;
  if (capture.options.capture !== 'hold') throw new Error(`${name}: requires --capture hold`);
  const { samples, options } = capture;
  if (!samples.every((sample, i) => Number.isFinite(sample.ms) &&
      Number.isFinite(sample.velocity) && Number.isFinite(sample.reference_rad_s) &&
      Number.isFinite(sample.mean_velocity_rad_s) &&
      (i === 0 || sample.ms > samples[i - 1].ms))) throw new Error(`${name}: invalid sample sequence`);
  const release = 350 + options.pulse_ms;
  const end = samples.at(-1).ms;
  const finalWindow = Math.max(release + 500, end - 2000);
  const released = samples.filter(sample => sample.ms > release + 500);
  const held = released.filter(sample => sample.state === 2);
  const final = samples.filter(sample => sample.ms >= finalWindow);
  const finalHeld = final.filter(sample => sample.state === 2);
  const reference = finalHeld.length ? finalHeld.at(-1).reference_rad_s : 0;
  const finalMean = windowMean(samples, finalWindow, end);
  const initialMean = windowMean(samples, Math.max(350, release - 500), release);
  const references = [...new Set(held.map(sample => sample.reference_rad_s))];
  const entry = {
    file: name,
    status: capture.status,
    drive_target_rad_s: options.pulse_velocity_rad_s,
    pre_release_mean_rad_s: initialMean,
    final_mean_rad_s: finalMean,
    final_reference_rad_s: reference,
    final_error_rad_s: finalHeld.length ? finalMean - reference : null,
    release_observation_s: Math.max(0, end - release) / 1000,
    observed_reference_count: references.length,
    observed_references_rad_s: references,
    peak_speed_rad_s: Math.max(...samples.map(sample => Math.abs(sample.velocity))),
    peak_command_ma: Math.max(...samples.map(sample => Math.abs(sample.command_ma))),
    final_states: [...new Set(final.map(sample => sample.state))],
    final_travel_rad: final.at(-1).angle - final[0].angle,
    deadline_misses: capture.metrics.app_control_deadline_miss_count,
    control_max_us: capture.metrics.runtime_control_max_cycles / capture.metrics.SystemCoreClock * 1e6,
    disturbance: { current_ma: options.disturb_ma, start_ms: options.disturb_start_ms,
      duration_ms: options.disturb_ms },
  };
  captures.push(entry);
  console.log(`${name}: before=${initialMean?.toFixed(3)} end=${finalMean?.toFixed(3)} ref=${reference.toFixed(3)} references=${references.length} status=${capture.status}`);
}
const report = {
  methodology: 'Time-weighted firmware window means; pre-release last 500 ms, final last 2 s. Reference count excludes first 500 ms after release. Electrical disturbances emulate applied shaft torque; they do not verify tactile feel.',
  captures,
};
if (option('--out')) writeFileSync(option('--out'), JSON.stringify(report, null, 2) + '\n');
