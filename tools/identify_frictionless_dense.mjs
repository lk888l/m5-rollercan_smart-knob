// Offline dense-capture identification. No motor, probe or process APIs.
import { readFileSync, readdirSync, statSync, mkdirSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import assert from 'node:assert/strict';

const LAGS_MS = [0, 1, 2];
const ALPHA = 1 / 9;
const WARMUP_MS = 50;
const DEFAULT_SETS = [[12], [7, 12], Array.from({ length: 24 }, (_, i) => i + 1)];
const mean = values => values.reduce((sum, value) => sum + value, 0) / values.length;
const norm = values => Math.sqrt(values.reduce((sum, value) => sum + value * value, 0));
const phaseDifference = (a, b) => Math.atan2(Math.sin(a - b), Math.cos(a - b));

export function prepareDenseCapture(capture, source, harmonics, { windowMs = 10 } = {}) {
  const fail = reason => ({ source, rows: [], reasons: [reason] });
  if (capture.status !== 2) return fail('Capture did not complete normally');
  if (capture.options?.dense !== 1 && capture.options?.dense !== true) return fail('Not an ABI 5 dense capture');
  if (capture.metrics?.app_control_deadline_miss_count > 0) return fail('Control deadline misses recorded');
  const target = capture.options?.pulse_velocity_rad_s;
  const end = 350 + capture.options?.pulse_ms;
  const samples = capture.samples;
  if (!Number.isFinite(target) || !target || !Number.isFinite(end) || !Array.isArray(samples) || samples.length < 60) {
    return fail('Invalid speed PI target, timing or sample count');
  }
  if (samples.some((row, i) => !['ms', 'angle', 'velocity', 'filtered_command_ma'].every(key => Number.isFinite(row[key]))
    || !Number.isInteger(row.ms) || (i && row.ms <= samples[i - 1].ms))) return fail('Invalid or unordered dense telemetry');
  const byTime = new Map(samples.map(row => [row.ms, row]));
  const direction = Math.sign(target);
  const firstMs = samples[0].ms;
  const sine = harmonics.map(h => Math.sin(h * samples[0].angle));
  const cosine = harmonics.map(h => Math.cos(h * samples[0].angle));
  const denseRows = [], rejected = {};
  let interpolatedMilliseconds = 0;
  function exclude(reason) { rejected[reason] = (rejected[reason] ?? 0) + 1; }
  for (let i = 1; i < samples.length; ++i) {
    const previous = samples[i - 1], row = samples[i];
    const dtMs = row.ms - previous.ms;
    if (dtMs > 5) return fail('Gap exceeds 5 ms; cannot reconstruct the angle filters reliably');
    // Reconstruct only the angle-basis filters. Velocity and current already
    // carry their firmware filters and must not be filtered a second time.
    for (let step = 1; step <= dtMs; ++step) {
      const angle = previous.angle + (row.angle - previous.angle) * step / dtMs;
      harmonics.forEach((h, j) => {
        sine[j] += ALPHA * (Math.sin(h * angle) - sine[j]);
        cosine[j] += ALPHA * (Math.cos(h * angle) - cosine[j]);
      });
    }
    if (dtMs > 1) interpolatedMilliseconds += dtMs - 1;
    if (previous.ms < firstMs + WARMUP_MS || previous.ms < 550 || row.ms >= end) {
      exclude('filter_warmup_or_outside_pi'); continue;
    }
    if (dtMs !== 1 || LAGS_MS.some(lag => !byTime.has(row.ms - lag))) {
      exclude('noncontiguous_derivative_or_lag'); continue;
    }
    if ([row.velocity, previous.velocity].some(v => direction * v < 0.6)) {
      exclude('slow_or_reversed'); continue;
    }
    denseRows.push({ source, ms: row.ms, angle: row.angle, direction, target_velocity_rad_s: target,
      // Backward difference is the filtered acceleration during this 1 ms
      // interval. Lag selection is an effective discrete timing alignment.
      features: [direction, row.velocity, (row.velocity - previous.velocity) * 1000, 1,
        ...harmonics.flatMap((h, j) => [sine[j], cosine[j]])],
      currents: LAGS_MS.map(lag => byTime.get(row.ms - lag).filtered_command_ma),
    });
  }
  const rows = [], angleBins = Array(32).fill(0);
  let bucket = [];
  for (const row of denseRows) {
    if (bucket.length && row.ms !== bucket.at(-1).ms + 1) {
      rejected.incomplete_averaging_window = (rejected.incomplete_averaging_window ?? 0) + bucket.length;
      bucket = [];
    }
    bucket.push(row);
    if (bucket.length !== windowMs) continue;
    const features = row.features.map((unused, i) => mean(bucket.map(sample => sample.features[i])));
    const currents = LAGS_MS.map((unused, i) => mean(bucket.map(sample => sample.currents[i])));
    // Averaging adjacent 1 ms velocity differences telescopes exactly to
    // (v_end - v_start) / window_seconds. Average ALL other regressors and
    // lagged currents over that same interval to preserve the torque balance.
    rows.push({ source, ms: row.ms, direction, target_velocity_rad_s: target, features, currents });
    for (const sample of bucket) {
      const wrapped = ((sample.angle % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
      ++angleBins[Math.min(31, Math.floor(wrapped / (2 * Math.PI) * 32))];
    }
    bucket = [];
  }
  if (bucket.length) rejected.incomplete_averaging_window = (rejected.incomplete_averaging_window ?? 0) + bucket.length;
  return { source, target_velocity_rad_s: target, rows, reasons: rows.length ? [] : ['No qualified dense intervals'],
    rejected_intervals: rejected, interpolated_milliseconds: interpolatedMilliseconds,
    angle_bin_samples: angleBins, window_ms: windowMs,
    mechanical_angle_span_rad: samples.at(-1).angle - samples[0].angle };
}

function sufficientStats(rows, width) {
  const gram = Array.from({ length: width }, () => Array(width).fill(0));
  const rhs = LAGS_MS.map(() => Array(width).fill(0));
  const yy = LAGS_MS.map(() => 0);
  for (const row of rows) {
    for (let i = 0; i < width; ++i) {
      for (let j = 0; j <= i; ++j) gram[i][j] += row.features[i] * row.features[j];
      for (let lag = 0; lag < LAGS_MS.length; ++lag) rhs[lag][i] += row.features[i] * row.currents[lag];
    }
    row.currents.forEach((value, lag) => { yy[lag] += value * value; });
  }
  for (let i = 0; i < width; ++i) for (let j = 0; j < i; ++j) gram[j][i] = gram[i][j];
  return { n: rows.length, gram, rhs, yy };
}

function combineStats(items, width) {
  const total = sufficientStats([], width);
  for (const item of items) {
    total.n += item.n;
    for (let i = 0; i < width; ++i) {
      for (let j = 0; j < width; ++j) total.gram[i][j] += item.gram[i][j];
      for (let lag = 0; lag < 3; ++lag) total.rhs[lag][i] += item.rhs[lag][i];
    }
    item.yy.forEach((value, lag) => { total.yy[lag] += value; });
  }
  return total;
}

function solveStats(stats, indices, lag) {
  if (stats.n <= indices.length * 2) throw new Error('Insufficient independent training samples');
  const scales = indices.map(index => Math.sqrt(stats.gram[index][index]));
  if (scales.some(value => !Number.isFinite(value) || value < 1e-10)) throw new Error('Unexcited model feature');
  const matrix = indices.map((index, i) => [
    ...indices.map((other, j) => stats.gram[index][other] / (scales[i] * scales[j])),
    stats.rhs[lag][index] / scales[i],
  ]);
  let minPivot = Infinity;
  for (let i = 0; i < indices.length; ++i) {
    let pivot = i;
    for (let j = i + 1; j < indices.length; ++j) if (Math.abs(matrix[j][i]) > Math.abs(matrix[pivot][i])) pivot = j;
    const pivotSize = Math.abs(matrix[pivot][i]);
    if (pivotSize < 1e-10) throw new Error('Rank-deficient/ill-conditioned angle coverage; collect more shaft phases');
    minPivot = Math.min(minPivot, pivotSize);
    [matrix[i], matrix[pivot]] = [matrix[pivot], matrix[i]];
    const divisor = matrix[i][i];
    for (let j = i; j <= indices.length; ++j) matrix[i][j] /= divisor;
    for (let row = 0; row < indices.length; ++row) {
      if (row === i) continue;
      const multiplier = matrix[row][i];
      for (let j = i; j <= indices.length; ++j) matrix[row][j] -= multiplier * matrix[i][j];
    }
  }
  return { coefficients: matrix.map((row, i) => row.at(-1) / scales[i]), min_scaled_pivot: minPivot };
}

function residualRms(stats, indices, coefficients, lag) {
  let sse = stats.yy[lag];
  for (let i = 0; i < indices.length; ++i) {
    sse -= 2 * coefficients[i] * stats.rhs[lag][indices[i]];
    for (let j = 0; j < indices.length; ++j) sse += coefficients[i] * coefficients[j] * stats.gram[indices[i]][indices[j]];
  }
  return Math.sqrt(Math.max(0, sse) / stats.n);
}

function modelIndices(harmonics, union, bias = true) {
  return [0, 1, 2, ...(bias ? [3] : []), ...harmonics.flatMap(h => {
    const i = union.indexOf(h); return [4 + 2 * i, 5 + 2 * i];
  })];
}

function describeFit(stats, harmonics, union, lag, bias = true) {
  const indices = modelIndices(harmonics, union, bias);
  try {
    const solution = solveStats(stats, indices, lag);
    const coefficients = solution.coefficients;
    const start = bias ? 4 : 3;
    const terms = harmonics.map((harmonic, i) => {
      const sinMa = coefficients[start + 2 * i], cosMa = coefficients[start + 2 * i + 1];
      return { harmonic, sin_ma: sinMa, cos_ma: cosMa, amplitude_ma: Math.hypot(sinMa, cosMa), phase_rad: Math.atan2(cosMa, sinMa) };
    });
    return { valid: true, harmonics, lag_ms: LAGS_MS[lag], sample_count: stats.n,
      coulomb_ma: coefficients[0], viscous_ua_per_rad_s: coefficients[1] * 1000,
      inertia_ma_per_rad_s2: coefficients[2], positive_inertia: coefficients[2] > 0,
      bias_ma: bias ? coefficients[3] : null,
      terms, residual_rms_ma: residualRms(stats, indices, coefficients, lag),
      min_scaled_pivot: solution.min_scaled_pivot,
      coefficients, indices };
  } catch (error) { return { valid: false, harmonics, lag_ms: LAGS_MS[lag], reason: error.message }; }
}

function crossValidate(groups, total, harmonics, union, lag) {
  const fit = describeFit(total, harmonics, union, lag);
  if (!fit.valid) return { ...fit, cross_validation_valid: false };
  if (groups.length < 4 || [-1, 1].some(direction => groups.filter(g => g.direction === direction).length < 2)) {
    return { ...fit, cross_validation_valid: false, reason: 'Leave-one-capture-out needs at least two captures in each direction' };
  }
  const folds = groups.map((heldOut, excludedIndex) => {
    const training = combineStats(groups.filter((g, i) => i !== excludedIndex).map(g => g.stats), total.gram.length);
    const trained = describeFit(training, harmonics, union, lag);
    return trained.valid ? { source: heldOut.source, valid: true, inertia_ma_per_rad_s2: trained.inertia_ma_per_rad_s2,
      held_out_rms_ma: residualRms(heldOut.stats, trained.indices, trained.coefficients, lag) }
      : { source: heldOut.source, valid: false, reason: trained.reason };
  });
  if (folds.some(fold => !fold.valid)) return { ...fit, cross_validation_valid: false, folds };
  const errors = folds.map(fold => fold.held_out_rms_ma);
  const meanError = mean(errors);
  const standardError = Math.sqrt(errors.reduce((sum, value) => sum + (value - meanError) ** 2, 0) / (errors.length - 1) / errors.length);
  return { ...fit, cross_validation_valid: true, folds,
    cv_rms_ma: Math.sqrt(mean(errors.map(value => value * value))),
    cv_mean_capture_rms_ma: meanError, cv_capture_rms_standard_error_ma: standardError,
    cv_max_capture_rms_ma: Math.max(...errors),
    cv_all_positive_inertia: folds.every(fold => fold.inertia_ma_per_rad_s2 > 0),
  };
}

function publicModel(model) {
  const { coefficients, indices, ...result } = model;
  return result;
}

export function cCoefficientTable(model) {
  if (!model?.valid) return null;
  const lines = model.terms.map(term => `    {${term.harmonic}U, ${(term.sin_ma / 1000).toExponential(9)}f, ${(term.cos_ma / 1000).toExponential(9)}f},`);
  return '// Diagnostic candidate only: A * sin(h * mechanical_angle + phase).\n'
    + '// Coefficients are amperes; validate physical motion before deployment.\n'
    + 'static const struct { unsigned harmonic; float sin_a; float cos_a; } angle_terms[] = {\n'
    + lines.join('\n') + '\n};';
}

export function analyzeDenseCaptures(captures, { harmonicSets = DEFAULT_SETS, windowMs = 10 } = {}) {
  if (!Number.isInteger(windowMs) || windowMs < 1 || windowMs > 50) throw new Error('windowMs must be an integer in 1..50');
  const union = [...new Set(harmonicSets.flat())].sort((a, b) => a - b);
  const width = 4 + union.length * 2;
  const prepared = captures.map(({ capture, source }) => prepareDenseCapture(capture, source, union, { windowMs }));
  function angleCoverage(direction) {
    const items = prepared.filter(item => item.rows.length && (!direction || Math.sign(item.target_velocity_rad_s) === direction));
    const sampleCounts = Array.from({ length: 32 }, (_, bin) => items.reduce((sum, item) => sum + item.angle_bin_samples[bin], 0));
    const captureCounts = Array.from({ length: 32 }, (_, bin) => items.filter(item => item.angle_bin_samples[bin] > 0).length);
    const covered = sampleCounts.filter(count => count > 0).length;
    return { bin_count: 32, covered_bins: covered, coverage_fraction: covered / 32,
      missing_bins: sampleCounts.flatMap((count, bin) => count ? [] : [bin]),
      sample_counts: sampleCounts, independent_capture_counts: captureCounts,
      minimum_capture_support_per_bin: Math.min(...captureCounts),
      leave_one_capture_minimum_covered_bins: items.length ? Math.min(...items.map(item =>
        sampleCounts.filter((count, bin) => count - item.angle_bin_samples[bin] > 0).length)) : 0 };
  }
  const coverage = { combined: angleCoverage(0), positive: angleCoverage(1), negative: angleCoverage(-1) };
  const fullMapSupported = coverage.combined.covered_bins === 32 && coverage.combined.minimum_capture_support_per_bin >= 2;
  const groups = prepared.filter(item => item.rows.length).map(item => ({ source: item.source,
    direction: Math.sign(item.target_velocity_rad_s), stats: sufficientStats(item.rows, width) }));
  const total = combineStats(groups.map(group => group.stats), width);
  const candidates = harmonicSets.flatMap(harmonics => LAGS_MS.map((unused, lag) => crossValidate(groups, total, harmonics, union, lag)));
  const eligible = candidates.filter(model => model.cross_validation_valid && model.positive_inertia && model.cv_all_positive_inertia);
  eligible.sort((a, b) => a.cv_rms_ma - b.cv_rms_ma);
  const best = eligible[0];
  // A one-standard-error-style rule is a complexity heuristic, not a formal
  // confidence interval: repeated captures may share temperature/systematics.
  const nearBest = best ? eligible.filter(model => model.cv_rms_ma <= best.cv_rms_ma + best.cv_capture_rms_standard_error_ma) : [];
  nearBest.sort((a, b) => a.harmonics.length - b.harmonics.length || a.cv_rms_ma - b.cv_rms_ma || a.lag_ms - b.lag_ms);
  const selected = nearBest[0];
  let directions = null, consistency = null, lagAssessment = null;
  if (selected) {
    const lag = LAGS_MS.indexOf(selected.lag_ms);
    directions = Object.fromEntries([['positive', 1], ['negative', -1]].map(([name, direction]) => [name,
      describeFit(combineStats(groups.filter(group => group.direction === direction).map(group => group.stats), width), selected.harmonics, union, lag, false)]));
    const positive = directions.positive, negative = directions.negative;
    if (positive.valid && negative.valid) consistency = {
      inertia_relative_difference: (positive.inertia_ma_per_rad_s2 - negative.inertia_ma_per_rad_s2)
        / ((positive.inertia_ma_per_rad_s2 + negative.inertia_ma_per_rad_s2) / 2),
      terms: positive.terms.map((term, i) => ({ harmonic: term.harmonic,
        sin_difference_ma: term.sin_ma - negative.terms[i].sin_ma,
        cos_difference_ma: term.cos_ma - negative.terms[i].cos_ma,
        phase_difference_rad: phaseDifference(term.phase_rad, negative.terms[i].phase_rad),
        amplitude_difference_ma: term.amplitude_ma - negative.terms[i].amplitude_ma })),
    };
    const lagModels = eligible.filter(model => model.harmonics.join(',') === selected.harmonics.join(',')).sort((a, b) => a.cv_rms_ma - b.cv_rms_ma);
    const threshold = Math.max(0.1, lagModels[0].cv_rms_ma * 0.02);
    lagAssessment = { convention: 'filtered_command_ma(t - lag_ms) explains velocity/acceleration/angle basis at t',
      near_best_lags_ms: lagModels.filter(model => model.cv_rms_ma - lagModels[0].cv_rms_ma <= threshold).map(model => model.lag_ms),
      indistinguishability_threshold_ma: threshold,
      cv_errors: lagModels.map(model => ({ lag_ms: model.lag_ms, cv_rms_ma: model.cv_rms_ma })),
    };
    lagAssessment.ambiguous = lagAssessment.near_best_lags_ms.length > 1;
  }
  const warnings = [];
  if (!selected) warnings.push('No model passes capture-level cross-validation with positive inertia; do not deploy a coefficient table');
  if (selected?.coulomb_ma < 0 || selected?.viscous_ua_per_rad_s < 0) warnings.push('Negative fitted friction coefficient suggests model mismatch; do not transfer the loss parameters blindly');
  if (selected?.min_scaled_pivot < 1e-5) warnings.push('Weak model conditioning; collect more angles before trusting individual terms');
  if (lagAssessment?.ambiguous) warnings.push('Delay is weakly identified; differences below the reported threshold do not establish a physical latency');
  if (!fullMapSupported) warnings.push('Full angular map lacks repeated support in every 32-bin sector; high-order coefficient export is blocked');
  const exportAllowed = selected && (selected.harmonics.length <= 2 || fullMapSupported);
  return { analysis_version: 1, purpose: 'Offline diagnostic candidate; never writes firmware or motor settings',
    filter: { alpha: ALPHA, step_ms: 1, discarded_initial_ms_per_capture: WARMUP_MS, regression_window_ms: windowMs },
    input_capture_count: captures.length, accepted_capture_count: groups.length, accepted_sample_count: total.n,
    selection_rule: 'Require positive full-fit and leave-one-capture-out inertia; compare held-out RMSE and prefer fewer harmonics within the best model capture-error standard-error heuristic',
    candidates: candidates.map(publicModel), minimum_cv_model: best ? publicModel(best) : null,
    selected_model: selected ? publicModel(selected) : null,
    by_direction: directions ? Object.fromEntries(Object.entries(directions).map(([key, model]) => [key, publicModel(model)])) : null,
    direction_consistency: consistency, lag_assessment: lagAssessment,
    angular_coverage: coverage, full_map_export_supported: fullMapSupported,
    c_coefficients: exportAllowed ? cCoefficientTable(selected) : null, warnings,
    captures: prepared.map(({ rows, ...metadata }) => ({ ...metadata, accepted_samples: rows.length })),
    limitations: [
      'Cross-validation holds out entire captures, not correlated neighboring millisecond samples. It does not replace new independent physical releases.',
      'Same-alpha filtering removes the main velocity-filter mismatch, but unknown current-loop dynamics and one-step discretization can shift the effective best lag.',
      'The initial angle-filter history is reconstructed from the first angle; discarding 50 ms attenuates but cannot exactly remove its unknown initial error.',
      'Short captures can poorly separate low harmonics, friction and bias. Directional fits omit independent bias because it is collinear with Coulomb friction.',
      'M is 1000 * J / Kt in mA/(rad/s^2); it is not physical inertia without a motor torque constant.',
      'A large harmonic set can absorb sensor distortion, timing error or noise. Coefficient export is a diagnostic artifact, not automatic approval to use it.',
    ],
  };
}

export function selfTestDense() {
  const truth = { Fc: 23, B: 0.254, M: 0.14, bias: 0.13, terms: [[7, 4, 2], [12, -5.5, -15]], lag: 1 };
  const captures = Array.from({ length: 12 }, (_, run) => {
    const target = [3, -3, 6, -6, 12, -12][run % 6];
    const raw = [], balance = [], filteredVelocity = [], filteredCurrent = [];
    let angle = run * 0.771, previous = 0, vf = 0, cf = 0;
    for (let ms = 0; ms < 900; ++ms) {
      const velocity = target + 0.35 * Math.sin(ms * 0.023 + run) + 0.2 * Math.cos(ms * 0.037 - run);
      angle += velocity * 0.001;
      const current = truth.Fc * Math.sign(target) + truth.B * velocity + truth.M * (velocity - previous) * 1000
        + truth.terms.reduce((sum, [h, s, c]) => sum + s * Math.sin(h * angle) + c * Math.cos(h * angle), 0) + truth.bias;
      previous = velocity; vf += ALPHA * (velocity - vf);
      raw.push({ angle, velocity }); balance.push(current); filteredVelocity.push(vf);
    }
    // The command is issued one step before its modeled torque acts.
    for (let ms = 0; ms < 899; ++ms) {
      cf += ALPHA * (balance[ms + truth.lag] - cf); filteredCurrent.push(cf);
    }
    const samples = Array.from({ length: 384 }, (_, i) => { const k = 500 + i; return {
      ms: 2500 + i, angle: raw[k].angle, velocity: filteredVelocity[k], filtered_command_ma: filteredCurrent[k],
    }; });
    return { source: `synthetic-${run}`, capture: { options: { dense: 1, pulse_velocity_rad_s: target, pulse_ms: 5000 },
      status: 2, metrics: { app_control_deadline_miss_count: 0 }, samples } };
  });
  const report = analyzeDenseCaptures(captures);
  assert.ok(report.selected_model, 'a physically qualified model should be selected');
  assert.deepEqual(report.selected_model.harmonics, [7, 12]);
  assert.equal(report.selected_model.lag_ms, truth.lag);
  assert.ok(Math.abs(report.selected_model.inertia_ma_per_rad_s2 - truth.M) < 0.001);
  assert.ok(Math.abs(report.selected_model.coulomb_ma - truth.Fc) < 0.02);
  for (let i = 0; i < truth.terms.length; ++i) {
    assert.ok(Math.abs(report.selected_model.terms[i].sin_ma - truth.terms[i][1]) < 0.03);
    assert.ok(Math.abs(report.selected_model.terms[i].cos_ma - truth.terms[i][2]) < 0.03);
  }
  assert.ok(report.selected_model.cv_rms_ma < 0.03);
  const wrongLag = report.candidates.find(model => model.harmonics.join(',') === '7,12' && model.lag_ms === 0);
  assert.ok(wrongLag.cv_rms_ma > report.selected_model.cv_rms_ma * 5);
  assert.equal(prepareDenseCapture({ ...captures[0].capture, options: { ...captures[0].capture.options, dense: 0 } }, 'legacy', [12]).rows.length, 0);
  const stopped = structuredClone(captures[0].capture); stopped.samples[100].velocity = 0;
  assert.equal(prepareDenseCapture(stopped, 'stopped', [12]).rejected_intervals.slow_or_reversed, 2);
  const singleAngle = captures.map(item => ({ ...item, capture: { ...item.capture,
    samples: item.capture.samples.map(row => ({ ...row, angle: row.angle % 0.1 })) } }));
  const poorCoverage = analyzeDenseCaptures(singleAngle, { harmonicSets: [[12]] });
  assert.equal(poorCoverage.full_map_export_supported, false);
  return { passed: true, selected_harmonics: report.selected_model.harmonics, selected_lag_ms: report.selected_model.lag_ms,
    recovered_inertia: report.selected_model.inertia_ma_per_rad_s2, held_out_rms_ma: report.selected_model.cv_rms_ma,
    coefficients: report.selected_model.terms };
}

function main(argv) {
  if (argv.includes('--self-test')) { console.log(JSON.stringify(selfTestDense(), null, 2)); return; }
  let input, output, harmonicSets, windowMs = 10;
  for (let i = 0; i < argv.length; ++i) {
    const arg = argv[i];
    if (arg === '--help') {
      console.log('Usage: node tools/identify_frictionless_dense.mjs <capture-directory|capture.json> [--harmonics 7,12] [--window-ms 10] [--out report.json]\nDefault compares 12 vs 7,12 vs 1..24 and lag 0/1/2 ms using capture-level cross-validation and matched 10 ms interval averages.\n--self-test verifies a synthetic known physical model. No hardware is accessed.'); return;
    }
    if (arg === '--out' && argv[i + 1] && !argv[i + 1].startsWith('--')) { output = resolve(argv[++i]); continue; }
    if (arg === '--window-ms' && argv[i + 1] && !argv[i + 1].startsWith('--')) { windowMs = Number(argv[++i]); continue; }
    if (arg === '--harmonics' && argv[i + 1] && !argv[i + 1].startsWith('--')) {
      const values = argv[++i].split(',').map(Number);
      if (!values.length || values.some(h => !Number.isInteger(h) || h < 1 || h > 64) || new Set(values).size !== values.length) throw new Error('Harmonics must be distinct integers in 1..64');
      harmonicSets = [values.sort((a, b) => a - b)]; continue;
    }
    if (!arg.startsWith('--') && !input) { input = resolve(arg); continue; }
    throw new Error(`Unexpected or incomplete option: ${arg}`);
  }
  if (!input) throw new Error('Capture path required; use --help');
  const paths = statSync(input).isDirectory() ? readdirSync(input).filter(name => /^capture-.*\.json$/i.test(name)).sort().map(name => join(input, name)) : [input];
  if (!paths.length) throw new Error('No capture-*.json files found');
  const report = analyzeDenseCaptures(paths.map(source => ({ source, capture: JSON.parse(readFileSync(source, 'utf8')) })), { harmonicSets, windowMs });
  if (output) { mkdirSync(dirname(output), { recursive: true }); writeFileSync(output, JSON.stringify(report, null, 2) + '\n'); }
  console.log(JSON.stringify(output ? { report: output, accepted_captures: report.accepted_capture_count, selected: report.selected_model,
    direction_consistency: report.direction_consistency, lag_assessment: report.lag_assessment,
    angular_coverage: report.angular_coverage, warnings: report.warnings } : report, null, 2));
}

if (process.argv[1] && pathToFileURL(resolve(process.argv[1])).href === import.meta.url) main(process.argv.slice(2));
