// Offline analysis only: read diagnostic captures and print JSON to stdout.
import { readFileSync } from 'node:fs';
import { basename, extname, resolve } from 'node:path';
import { pathToFileURL } from 'node:url';

const RELEASE_DELAY_MS = 350;
const STOP_SPEED_RAD_S = 0.5;
const STOP_HOLD_MS = 200;
const LATE_DELAY_MS = 500;
const NORMAL_CAPACITY = 128;
const DENSE_CAPACITY = 384;
const peak = rows => rows.length ? Math.max(...rows) : null;
const median = values => {
  const sorted = [...values].sort((a, b) => a - b);
  const middle = Math.floor(sorted.length / 2);
  return sorted.length ? (sorted[middle] + sorted[Math.floor((sorted.length - 1) / 2)]) / 2 : null;
};

export function summarizeCapture(capture, sourcePath) {
  const path = resolve(sourcePath).replaceAll('\\', '/');
  const options = capture?.options ?? {};
  const rows = Array.isArray(capture?.samples) ? capture.samples : [];
  const errors = [];
  const dense = options.dense === 1;
  const capacity = dense ? DENSE_CAPACITY : NORMAL_CAPACITY;
  const currentField = dense ? 'filtered_command_ma' : 'command_ma';
  const pulseValid = Number.isInteger(options.pulse_ms) && options.pulse_ms >= 0;
  const releaseMs = pulseValid ? RELEASE_DELAY_MS + options.pulse_ms : null;
  const durationValid = Number.isInteger(options.duration_ms) && options.duration_ms > 0;
  const expectedPeriod = durationValid ? (dense ? 1 : Math.ceil(options.duration_ms / NORMAL_CAPACITY)) : null;
  const expectedCount = durationValid ? (dense ? DENSE_CAPACITY : Math.ceil(options.duration_ms / expectedPeriod)) : null;
  const expectedStart = durationValid && dense ? options.duration_ms - DENSE_CAPACITY : 0;
  const deadlineMisses = capture?.metrics?.app_control_deadline_miss_count;

  if (capture?.status !== 2) errors.push('status must be 2 (normal completion)');
  if (options.dense !== 0 && options.dense !== 1) errors.push('options.dense must be 0 or 1');
  if (!Number.isInteger(options.mode) || options.mode < 0 || options.mode > 0xFFFFFFFF) {
    errors.push('options.mode must be an unsigned integer');
  }
  if (!pulseValid || !durationValid || releaseMs >= options.duration_ms || expectedStart < 0) {
    errors.push('pulse_ms and duration_ms must define a release before capture end');
  }
  if (rows.length < 2 || rows.length > capacity || rows.length !== expectedCount) {
    errors.push(`sample count ${rows.length} does not match expected ${dense ? 'dense' : 'normal'} count ${expectedCount}`);
  }
  if (capture?.count !== undefined && capture.count !== rows.length) {
    errors.push('recorded count does not match samples.length');
  }
  if (!Number.isInteger(deadlineMisses) || deadlineMisses < 0) {
    errors.push('deadline miss count must be a nonnegative integer');
  }

  let samplesFinite = true;
  for (let index = 0; index < rows.length; ++index) {
    const row = rows[index];
    if (!row || typeof row !== 'object' || Array.isArray(row)
        || !['ms', 'angle', 'velocity', currentField].every(key => Number.isFinite(row[key]))
        || !Object.values(row).every(Number.isFinite)) {
      errors.push(`sample ${index} has missing or nonfinite numeric fields`);
      samplesFinite = false;
      break;
    }
    if (!Number.isInteger(row.ms) || row.ms < 0 || row.ms >= options.duration_ms) {
      errors.push(`sample ${index} timestamp is outside the capture interval`);
    }
  }

  const intervals = samplesFinite ? rows.slice(1).map((row, index) => row.ms - rows[index].ms) : [];
  if (intervals.some(interval => interval <= 0)) errors.push('sample times must increase strictly');
  if (intervals.some(interval => interval !== expectedPeriod)) {
    errors.push('actual sample intervals differ from the expected diagnostic sampling period');
  }
  if (capture?.period_ms !== undefined && capture.period_ms !== expectedPeriod) {
    errors.push('recorded period_ms differs from the expected sampling period');
  }
  if (samplesFinite && rows.length && (rows[0].ms !== expectedStart
      || options.duration_ms - rows.at(-1).ms > expectedPeriod)) {
    errors.push('capture does not cover the expected first and last sample windows');
  }

  const result = {
    name: basename(path, extname(path)),
    path,
    mode: Number.isInteger(options.mode) ? options.mode : null,
    valid: errors.length === 0,
    validation_errors: errors,
    status: capture?.status ?? null,
    count: rows.length,
    expected_count: expectedCount,
    dense,
    current_source: currentField,
    first_sample_ms: samplesFinite && rows.length ? rows[0].ms : null,
    last_sample_ms: samplesFinite && rows.length ? rows.at(-1).ms : null,
    sample_period_ms: median(intervals),
    expected_period_ms: expectedPeriod,
    sample_interval_min_ms: intervals.length ? Math.min(...intervals) : null,
    sample_interval_max_ms: peak(intervals),
    release_ms: releaseMs,
    postrelease_peak_speed: null,
    settle_ms: null,
    late_span_deg: null,
    late_peak_current_ma: null,
    late_peak_filtered_current_ma: null,
    reversals: null,
    deadline_miss_count: Number.isInteger(deadlineMisses) ? deadlineMisses : null,
  };
  if (!result.valid) return result;

  const coast = rows.filter(row => row.ms >= releaseMs);
  const late = rows.filter(row => row.ms >= releaseMs + LATE_DELAY_MS);
  let settleIndex = 0;
  let previousDirection = 0;
  let reversals = 0;
  for (let index = 0; index < coast.length; ++index) {
    const velocity = coast[index].velocity;
    if (Math.abs(velocity) >= STOP_SPEED_RAD_S) settleIndex = index + 1;
    if (Math.abs(velocity) > STOP_SPEED_RAD_S) {
      const direction = Math.sign(velocity);
      if (previousDirection !== 0 && direction !== previousDirection) ++reversals;
      previousDirection = direction;
    }
  }
  const settled = coast[settleIndex];
  return {
    ...result,
    postrelease_peak_speed: peak(coast.map(row => Math.abs(row.velocity))),
    settle_ms: settled && coast.at(-1).ms - settled.ms >= STOP_HOLD_MS
      ? settled.ms - releaseMs : null,
    late_span_deg: late.length
      ? (Math.max(...late.map(row => row.angle)) - Math.min(...late.map(row => row.angle))) * 180 / Math.PI
      : null,
    late_peak_current_ma: dense ? null : peak(late.map(row => Math.abs(row.command_ma))),
    late_peak_filtered_current_ma: dense ? peak(late.map(row => Math.abs(row.filtered_command_ma))) : null,
    reversals,
  };
}

export function main(paths = process.argv.slice(2)) {
  if (paths.length === 1 && paths[0] === '--help') {
    console.log('node tools/summarize_coarse_release.mjs <capture.json> [capture.json ...]\n'
      + 'Reads normal or dense motor_diagnostics captures and prints JSON; does not modify files or access hardware.');
    return;
  }
  if (!paths.length) throw new Error('Supply one or more motor_diagnostics JSON file paths.');
  const runs = paths.map(path => summarizeCapture(JSON.parse(readFileSync(path, 'utf8')), path));
  const summary = {
    valid: runs.every(run => run.valid),
    definitions: {
      release_ms: '350 + options.pulse_ms',
      velocity_source: 'velocity: instantaneous recorded fast-filtered velocity, in rad/s',
      postrelease_peak_speed: 'Maximum absolute recorded velocity at sample times >= release_ms, in rad/s',
      settle_ms: 'First sampled time after release with all subsequent samples abs(velocity)<0.5 rad/s, '
        + 'and at least 200 ms remaining through the last sample; measured relative to release; null if unconfirmed',
      late_window: 'Sample times >= release_ms + 500 ms through capture end',
      late_span_deg: 'Maximum minus minimum sampled angle in the late window, converted to degrees',
      late_peak_current_ma: 'Maximum absolute command_ma at sampled times in the late window',
      late_peak_filtered_current_ma: 'Dense captures only: maximum absolute filtered_command_ma at sampled '
        + 'times in the late window; this is the filtered command, not the instantaneous command',
      reversals: 'Post-release velocity sign changes after omitting samples with abs(velocity)<=0.5 rad/s',
      deadline_miss_count: 'Cumulative app_control_deadline_miss_count since firmware boot, not a per-run delta',
      sampling: 'Each run reports actual median/minimum/maximum sample intervals in ms. '
        + 'Peaks are sampled values, not true between-sample maxima. Settling time is limited by sampling resolution; '
        + 'unsampled reversals or transients cannot be excluded. Dense captures cover only the last 384 ms; '
        + 'metrics are restricted to the recorded interval and can omit earlier post-release motion. '
        + 'Missing late-window data yields null.',
      validation: 'Requires normal completion status 2, finite numeric samples, full expected sample count, '
        + 'strictly increasing timestamps and actual intervals equal to ceil(duration_ms/128) for ordinary captures '
        + 'or 1 ms for the final 384 samples of dense captures. '
        + 'Invalid captures retain metadata but have null motion metrics.',
    },
    runs,
  };
  console.log(JSON.stringify(summary, null, 2));
  if (!summary.valid) process.exitCode = 1;
}

if (process.argv[1] && import.meta.url === pathToFileURL(resolve(process.argv[1])).href) {
  try { main(); } catch (error) { console.error(error.message); process.exitCode = 1; }
}
