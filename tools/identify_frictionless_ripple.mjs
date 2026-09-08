// Offline model identification only: no probe, actuator, flash or settings access.
// All three extra regressors must be filtered at 1 kHz in the firmware with
// the same 8 ms filter as velocity. Sparse host captures cannot recreate them.
import { readFileSync, readdirSync, statSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { pathToFileURL } from 'node:url';

const FILTERED_FIELDS = ['mean_filtered_command_ma', 'mean_filtered_sin12', 'mean_filtered_cos12'];
const MODEL = 'filtered_current_ma = Fc * sign(target_velocity) + B * mean_filtered_velocity + M * delta_filtered_velocity / window_s + S * mean_filtered_sin12 + C * mean_filtered_cos12 [+ bias]';
const weightedMean = (values, weights) => values.reduce((sum, value, i) => sum + value * weights[i], 0)
  / weights.reduce((sum, value) => sum + value, 0);
const wrapPhase = value => Math.atan2(Math.sin(value), Math.cos(value));

export function collectRippleRows(capture, source = '', { skipMs = 2000, minSpeed = 0.6 } = {}) {
  const target = capture.options?.pulse_velocity_rad_s;
  const pulseEnd = 350 + capture.options?.pulse_ms;
  const samples = capture.samples ?? [];
  const reject = reason => ({ source, target_velocity_rad_s: target, rows: [], reasons: [reason], rejected_intervals: {} });
  if (capture.status !== 2) return reject(`Diagnostic status ${capture.status}; expected normal completion`);
  if (capture.metrics?.app_control_deadline_miss_count > 0) return reject('Control deadline misses were recorded');
  if (!Number.isFinite(target) || target === 0 || !Number.isFinite(pulseEnd)) return reject('Invalid speed PI target or timing');
  if (!Array.isArray(samples) || samples.length < 2) return reject('No interval samples');
  if (!samples.some(row => FILTERED_FIELDS.every(key => Number.isFinite(row[key])))) {
    return reject('Legacy capture: missing firmware-filtered current/sin12/cos12 telemetry; not reconstructed from sparse samples');
  }
  const rows = [];
  const rejected = {};
  const direction = Math.sign(target);
  function exclude(reason) { rejected[reason] = (rejected[reason] ?? 0) + 1; }
  for (let index = 1; index < samples.length; ++index) {
    const row = samples[index], previous = samples[index - 1];
    if (![row.ms, previous.ms].every(Number.isFinite) || row.ms <= previous.ms) {
      exclude('invalid_interval'); continue;
    }
    // Each mean covers (previous.ms, row.ms]. Keep the entire interval inside
    // the settled speed-PI phase, including its unobserved interior samples.
    if (previous.ms < skipMs || previous.ms < 550 || row.ms >= pulseEnd) {
      exclude('outside_settled_pi_window'); continue;
    }
    if (![row.velocity, previous.velocity, row.mean_velocity_rad_s,
      ...FILTERED_FIELDS.map(key => row[key])].every(Number.isFinite)) {
      exclude('missing_or_nonfinite_telemetry'); continue;
    }
    // Endpoint and average checks reject observed sticking and reversal. A
    // sub-window stop is not observable without extra firmware min/max data.
    if ([previous.velocity, row.velocity, row.mean_velocity_rad_s].some(value => direction * value < minSpeed)) {
      exclude('slow_or_reversed_interval'); continue;
    }
    if (Math.abs(row.mean_filtered_sin12) > 1.00001 || Math.abs(row.mean_filtered_cos12) > 1.00001) {
      exclude('invalid_filtered_angle_basis'); continue;
    }
    const intervalSeconds = (row.ms - previous.ms) / 1000;
    rows.push({
      source, target_velocity_rad_s: target, direction, ms: row.ms,
      interval_seconds: intervalSeconds,
      velocity: row.mean_velocity_rad_s,
      acceleration: (row.velocity - previous.velocity) / intervalSeconds,
      current: row.mean_filtered_command_ma,
      sin12: row.mean_filtered_sin12, cos12: row.mean_filtered_cos12,
    });
  }
  return { source, target_velocity_rad_s: target, rows,
    reasons: rows.length ? [] : ['No qualified settled moving intervals'], rejected_intervals: rejected };
}

// Re-orthogonalized, column-scaled QR avoids squaring the condition number in
// normal equations. No regularizer conceals a missing speed/direction basis.
function leastSquares(rows, keys) {
  if (rows.length <= keys.length) throw new Error('Too few qualified intervals');
  const weights = rows.map(row => row.interval_seconds);
  const totalWeight = weights.reduce((sum, value) => sum + value, 0);
  const columns = keys.map(key => rows.map((row, i) => feature(row, key) * Math.sqrt(weights[i])));
  const scales = columns.map(column => Math.hypot(...column));
  if (scales.some(scale => !Number.isFinite(scale) || scale < 1e-12)) throw new Error('A model feature has no excitation');
  const q = [], r = keys.map(() => keys.map(() => 0));
  for (let column = 0; column < keys.length; ++column) {
    const v = columns[column].map(value => value / scales[column]);
    for (let pass = 0; pass < 2; ++pass) {
      for (let earlier = 0; earlier < column; ++earlier) {
        const projection = v.reduce((sum, value, i) => sum + value * q[earlier][i], 0);
        r[earlier][column] += projection;
        for (let i = 0; i < v.length; ++i) v[i] -= projection * q[earlier][i];
      }
    }
    const norm = Math.hypot(...v);
    if (norm < 1e-8) throw new Error('Features are linearly dependent; add distinct speeds/directions or omit bias');
    r[column][column] = norm;
    q.push(v.map(value => value / norm));
  }
  const y = rows.map((row, i) => row.current * Math.sqrt(weights[i]));
  const rhs = q.map(column => column.reduce((sum, value, i) => sum + value * y[i], 0));
  const solution = keys.map(() => 0);
  for (let i = keys.length - 1; i >= 0; --i) {
    solution[i] = (rhs[i] - r[i].reduce((sum, value, j) => sum + (j > i ? value * solution[j] : 0), 0)) / r[i][i];
  }
  return { coefficients: Object.fromEntries(keys.map((key, i) => [key, solution[i] / scales[i]])),
    duration_seconds: totalWeight, minimum_relative_qr_diagonal: Math.min(...r.map((row, i) => row[i])) };
}

function feature(row, key) {
  return key === 'Fc' ? row.direction : key === 'B' ? row.velocity : key === 'M' ? row.acceleration
    : key === 'S' ? row.sin12 : key === 'C' ? row.cos12 : 1;
}

function residualStats(rows, coefficients) {
  const residuals = rows.map(row => row.current - Object.entries(coefficients)
    .reduce((sum, [key, value]) => sum + value * feature(row, key), 0));
  const weights = rows.map(row => row.interval_seconds);
  const mean = weightedMean(residuals, weights);
  return { sample_count: rows.length, residual_mean_ma: mean,
    residual_rms_ma: Math.sqrt(weightedMean(residuals.map(value => value * value), weights)),
    residual_std_ma: Math.sqrt(weightedMean(residuals.map(value => (value - mean) ** 2), weights)),
    residual_max_abs_ma: Math.max(...residuals.map(Math.abs)) };
}

export function fitRippleModel(rows, { withRipple = true, includeBias = false } = {}) {
  const keys = ['Fc', 'B', 'M', ...(withRipple ? ['S', 'C'] : []), ...(includeBias ? ['bias'] : [])];
  try {
    const result = leastSquares(rows, keys);
    const c = result.coefficients;
    const warnings = [];
    if (c.M <= 0) warnings.push('Nonpositive inertia coefficient: current timing, model or measurements are inconsistent');
    if (c.Fc < 0 || c.B < 0) warnings.push('Negative fitted loss: do not transfer these coefficients to firmware');
    if (result.minimum_relative_qr_diagonal < 1e-3) warnings.push('Weakly independent model features; coefficients may be poorly identified');
    return { valid: true, ...result, sample_count: rows.length,
      coulomb_ma: c.Fc, viscous_ua_per_rad_s: c.B * 1000,
      inertia_ma_per_rad_s2: c.M,
      ripple_sin12_ma: c.S ?? 0, ripple_cos12_ma: c.C ?? 0,
      ripple_amplitude_ma: withRipple ? Math.hypot(c.S, c.C) : 0,
      ripple_phase_rad: withRipple ? Math.atan2(c.C, c.S) : null,
      ripple_phase_convention: 'A * sin(12 * mechanical_angle_rad + phase)',
      bias_ma: c.bias ?? 0, ...residualStats(rows, c), warnings };
  } catch (error) {
    return { valid: false, sample_count: rows.length, reason: error.message };
  }
}

function compareModels(rows, includeBias) {
  const withoutRipple = fitRippleModel(rows, { withRipple: false, includeBias });
  const withRipple = fitRippleModel(rows, { withRipple: true, includeBias });
  const improvement = withoutRipple.valid && withRipple.valid && withoutRipple.residual_rms_ma > 0
    ? 1 - (withRipple.residual_rms_ma / withoutRipple.residual_rms_ma) ** 2 : null;
  return { without_ripple: withoutRipple, with_ripple: withRipple, residual_variance_reduction_fraction: improvement };
}

export function analyzeRippleCaptures(captures, options = {}) {
  const includeBias = options.includeBias ?? false;
  const collected = captures.map(({ capture, source }) => collectRippleRows(capture, source, options));
  const rows = collected.flatMap(item => item.rows);
  const combined = compareModels(rows, includeBias);
  const byDirection = Object.fromEntries([['positive', 1], ['negative', -1]].map(([name, direction]) =>
    [name, compareModels(rows.filter(row => row.direction === direction), false)]));
  const positive = byDirection.positive.with_ripple, negative = byDirection.negative.with_ripple;
  const perCapture = collected.map(({ rows: captureRows, ...metadata }) => ({ ...metadata,
    accepted_intervals: captureRows.length,
    ...(captureRows.length && combined.with_ripple.valid ? {
      combined_model_residuals: residualStats(captureRows, combined.with_ripple.coefficients),
      combined_without_ripple_residuals: combined.without_ripple.valid
        ? residualStats(captureRows, combined.without_ripple.coefficients) : null,
    } : {}),
  }));
  return {
    analysis_version: 1, purpose: 'Offline diagnostic identification; does not flash, actuate or save motor settings',
    model: MODEL, include_bias: includeBias,
    qualification: { skip_first_ms: options.skipMs ?? 2000, minimum_signed_endpoint_and_mean_speed_rad_s: options.minSpeed ?? 0.6 },
    accepted_intervals: rows.length, accepted_captures: collected.filter(item => item.rows.length).length,
    combined, by_direction: byDirection,
    direction_consistency: positive.valid && negative.valid ? {
      inertia_positive_ma_per_rad_s2: positive.inertia_ma_per_rad_s2,
      inertia_negative_ma_per_rad_s2: negative.inertia_ma_per_rad_s2,
      inertia_difference_relative_to_mean: (positive.inertia_ma_per_rad_s2 - negative.inertia_ma_per_rad_s2)
        / ((positive.inertia_ma_per_rad_s2 + negative.inertia_ma_per_rad_s2) / 2),
      ripple_sin12_difference_ma: positive.ripple_sin12_ma - negative.ripple_sin12_ma,
      ripple_cos12_difference_ma: positive.ripple_cos12_ma - negative.ripple_cos12_ma,
      ripple_phase_difference_rad: wrapPhase(positive.ripple_phase_rad - negative.ripple_phase_rad),
      ripple_amplitude_difference_ma: positive.ripple_amplitude_ma - negative.ripple_amplitude_ma,
    } : null,
    captures: perCapture,
    limitations: [
      'Linear filtering commutes with the constant-coefficient torque balance and interval integration; sign is treated as constant only in qualified motion.',
      'M is current-equivalent inertia (1000 * J / Kt), not physical inertia without the motor torque constant.',
      'One-direction bias is inseparable from Coulomb friction; each directional fit omits bias and its Fc absorbs it.',
      'Commanded current approximates torque current. Actuator delay, current-loop tracking error and unmodeled angle harmonics remain possible residuals.',
      'Endpoint/mean checks cannot exclude a stop hidden inside a capture interval; steady low-ripple validation remains necessary.',
      'Fit residuals are in-sample and correlated. Repeated captures and direction agreement are diagnostic checks, not proof of a deployable compensation map.',
    ],
  };
}

function main(argv) {
  let input, output, includeBias = false;
  for (let index = 0; index < argv.length; ++index) {
    const arg = argv[index];
    if (arg === '--help') {
      console.log('Usage: node tools/identify_frictionless_ripple.mjs <capture-directory|capture.json> [--bias] [--out report.json]\nOffline only. Requires firmware-filtered current/sin12/cos12 statistics; legacy captures are reported and skipped.');
      return;
    }
    if (arg === '--bias') { includeBias = true; continue; }
    if (arg === '--out' && argv[index + 1] && !argv[index + 1].startsWith('--')) { output = resolve(argv[++index]); continue; }
    if (!arg.startsWith('--') && !input) { input = resolve(arg); continue; }
    throw new Error(`Unexpected or incomplete argument: ${arg}`);
  }
  if (!input) throw new Error('A capture directory or JSON file is required; use --help');
  const paths = statSync(input).isDirectory()
    ? readdirSync(input).filter(name => /^capture-.*\.json$/i.test(name)).sort().map(name => join(input, name)) : [input];
  if (!paths.length) throw new Error('No capture-*.json files found');
  const captures = paths.map(source => ({ source, capture: JSON.parse(readFileSync(source, 'utf8')) }));
  const report = analyzeRippleCaptures(captures, { includeBias });
  if (output) { mkdirSync(dirname(output), { recursive: true }); writeFileSync(output, JSON.stringify(report, null, 2) + '\n'); }
  console.log(JSON.stringify(output ? { report: output, accepted_intervals: report.accepted_intervals,
    combined: report.combined, direction_consistency: report.direction_consistency } : report, null, 2));
}

if (process.argv[1] && pathToFileURL(resolve(process.argv[1])).href === import.meta.url) main(process.argv.slice(2));
