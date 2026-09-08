import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const directory = process.argv[2] ?? 'build/frictionless';
const mean = values => values.reduce((sum, value) => sum + value, 0) / values.length;
const results = [];
for (const file of readdirSync(directory).filter(name => name.endsWith('.json'))) {
  const data = JSON.parse(readFileSync(join(directory, file), 'utf8'));
  if (data.status !== 2 || !data.samples?.length || !data.options) continue;
  const releaseMs = 350 + data.options.pulse_ms;
  const coast = data.samples.filter(sample => sample.ms >= releaseMs);
  if (!coast.length) continue;
  // Require the remaining trace to stay near zero for at least 200 ms.
  // A momentary zero crossing during a rebound is not a settled shaft.
  const settled = coast.findIndex((sample, index) =>
    coast.at(-1).ms - sample.ms >= 200 &&
    coast.slice(index).every(row => Math.abs(row.velocity) < 0.2));
  const tail = coast.filter(sample => sample.ms >= coast.at(-1).ms - 200);
  const before = data.samples.filter(sample => sample.ms < releaseMs && sample.ms >= releaseMs - 200);
  results.push({
    file, options: data.options,
    release_time_ms: releaseMs,
    initial_speed_rad_s: before.length ? mean(before.map(row => row.velocity)) : coast[0].velocity,
    final_speed_rad_s: mean(tail.map(row => row.velocity)),
    observed_coast_s: (coast.at(-1).ms - releaseMs) / 1000,
    settled_after_s: settled < 0 ? null : (coast[settled].ms - releaseMs) / 1000,
    coast_travel_rad: coast.at(-1).angle - coast[0].angle,
    max_assist_ma: Math.max(...coast.map(row => Math.abs(row.command_ma))),
    control_deadline_misses: data.metrics.app_control_deadline_miss_count,
    max_control_step_us: data.metrics.SystemCoreClock
      ? data.metrics.runtime_control_max_cycles / data.metrics.SystemCoreClock * 1e6 : null,
  });
}
const summary = { stop_threshold_rad_s: 0.2, minimum_settled_duration_ms: 200, results };
writeFileSync(join(directory, 'validation-summary.json'), JSON.stringify(summary, null, 2));
console.log(JSON.stringify(results.filter(row => /final-|legacy-free-|long-coast-pos\.json|idle-final/.test(row.file)), null, 2));
