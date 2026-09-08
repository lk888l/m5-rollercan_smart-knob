// Serial, bounded release tests through motor_diagnostics.mjs; no flash/save operations.
// Node.js only. Import analyzeRelease() to inspect recorded or synthetic captures offline.
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const ANALYSIS_VERSION = 2;
const SKIP_RELEASE_MS = 200;
const SETTLING_MS = 350;
const MIN_COAST_SPEED_RATIO = 0.5;
const MAX_COAST_SPEED_RATIO = 1.5;
const mean = values => values.reduce((sum, value) => sum + value, 0) / values.length;
const square = value => value * value;
const round = value => Number(value.toFixed(6));

function writeJson(path, value) {
  mkdirSync(dirname(path), { recursive: true });
  writeFileSync(path, JSON.stringify(value, null, 2) + '\n');
}

function readJson(path) {
  return JSON.parse(readFileSync(path, 'utf8'));
}

/** Analyze the complete release curve after the initial control handover.
 * Positive slope/drift always means speeding up in the commanded direction.
 * Loss is dimensionless, normalized to the requested speed; the reference is
 * the measured speed immediately before release. Thus handover losses remain
 * visible even when the rotor stops within the skipped 200 ms. The final speed
 * is a window mean, not one sample.
 */
export function analyzeRelease(capture, expected = {}) {
  const options = capture.options ?? {};
  const target = Number(expected.speed_rad_s ?? options.pulse_velocity_rad_s);
  const pulseMs = Number(expected.pulse_ms ?? options.pulse_ms);
  const durationMs = Number(expected.duration_ms ?? options.duration_ms);
  const direction = Math.sign(target);
  const norm = Math.abs(target);
  const releaseMs = SETTLING_MS + pulseMs;
  const startMs = releaseMs + SKIP_RELEASE_MS;
  const reasons = [];
  if (capture.status !== 2) reasons.push(`diagnostic status ${capture.status}, expected 2`);
  const missed = capture.metrics?.app_control_deadline_miss_count;
  if (missed !== 0) reasons.push(`deadline miss count ${missed ?? 'unavailable'}, expected 0`);
  if (!Number.isFinite(target) || norm <= 0 || !Number.isFinite(pulseMs) || !Number.isFinite(durationMs)) {
    return { valid: false, qualified: false, reasons: [...reasons, 'invalid capture timing or target'], loss: null };
  }
  const samples = Array.isArray(capture.samples) ? capture.samples : [];
  const source = samples.length > 0 && samples.every(row => Number.isFinite(row.mean_velocity_rad_s))
    ? 'mean_velocity_rad_s' : 'velocity';
  if (samples.some(row => !Number.isFinite(row.ms) || !Number.isFinite(row[source]))) {
    reasons.push('non-finite sample time or velocity');
  }
  if (samples.some((row, index) => index > 0 && row.ms <= samples[index - 1].ms)) {
    reasons.push('sample times are not strictly increasing');
  }
  const rows = samples.filter(row => Number.isFinite(row.ms) && Number.isFinite(row[source]) && row.ms >= startMs)
    .map(row => ({ ms: row.ms, velocity: row[source] * direction }));
  if (rows.length < 8 || rows.at(-1).ms - rows[0].ms < 400) {
    return { valid: false, qualified: false, reasons: [...reasons, 'insufficient release samples'], loss: null };
  }
  // Prefer an explicit period, otherwise infer it from actual sample spacing;
  // different diagnostic ABIs have different capture-buffer capacities.
  const intervals = samples.slice(1).map((row, index) => row.ms - samples[index].ms)
    .filter(period => Number.isFinite(period) && period > 0).sort((a, b) => a - b);
  const configuredPeriod = Number(options.period_ms ?? capture.period_ms);
  const expectedPeriod = Number.isFinite(configuredPeriod) && configuredPeriod > 0 ? configuredPeriod
    : intervals.length ? intervals[Math.floor(intervals.length / 2)] : Math.ceil(durationMs / Math.max(1, samples.length));
  // Captures intentionally omit the final instant (Stop freezes the buffer).
  if (rows.at(-1).ms < durationMs - Math.max(150, 3 * expectedPeriod)) reasons.push('capture ended prematurely');
  const firstMs = rows[0].ms;
  const lastMs = rows.at(-1).ms;
  // Mean-velocity samples describe the window ending at row.ms. Exclude the
  // boundary sample and every window extending across release into feedback.
  const referenceRows = samples.filter(row => Number.isFinite(row.ms) && Number.isFinite(row[source])
    && row.ms >= releaseMs - 200 && row.ms < releaseMs && row.ms >= SETTLING_MS)
    .map(row => ({ ms: row.ms, velocity: row[source] * direction }));
  if (!referenceRows.length) {
    return { valid: false, qualified: false, reasons: [...reasons, 'no pre-release PI reference samples'], loss: null };
  }
  const postHandoverRows = rows.filter(row => row.ms < firstMs + 200);
  const finalRows = rows.filter(row => row.ms >= lastMs - 200);
  const reference = mean(referenceRows.map(row => row.velocity));
  const postHandoverReference = mean(postHandoverRows.map(row => row.velocity));
  const final = mean(finalRows.map(row => row.velocity));
  if (reference < Math.max(0.6, norm * 0.2)) reasons.push('pulse did not establish useful forward motion');
  const times = rows.map(row => (row.ms - firstMs) / 1000);
  const velocities = rows.map(row => row.velocity);
  const timeMean = mean(times);
  const velocityMean = mean(velocities);
  const slope = times.reduce((sum, t, i) => sum + (t - timeMean) * (velocities[i] - velocityMean), 0)
    / times.reduce((sum, t) => sum + square(t - timeMean), 0);
  const variance = mean(velocities.map(v => square(v - velocityMean)));
  const residualVariance = mean(velocities.map((v, i) => square(v - velocityMean - slope * (times[i] - timeMean))));
  const errors = velocities.map(v => (v - reference) / norm);
  const curveMse = mean(errors.map(square));
  const accelerationMse = mean(errors.map(error => square(Math.max(0, error))));
  const stopThreshold = Math.max(0.2, norm * 0.02);
  const stoppedFraction = mean(velocities.map(v => Math.abs(v) <= stopThreshold ? 1 : 0));
  const reversedFraction = mean(velocities.map(v => v < -stopThreshold ? 1 : 0));
  const stopped = finalRows.every(row => Math.abs(row.velocity) <= stopThreshold);
  const reversed = rows.some(row => row.velocity < -stopThreshold);
  const releaseSeconds = (lastMs - firstMs) / 1000;
  const slopeDrift = slope * releaseSeconds / norm;
  const normalizedVariance = variance / square(norm);
  const loss = curveMse + 2 * accelerationMse + 0.25 * square(slopeDrift)
    + 0.1 * normalizedVariance + 2 * stoppedFraction + 4 * reversedFraction
    + (stopped ? 1 : 0) + (reversed ? 1 : 0);
  // A valid measurement can describe an unusable coast. Numeric loss alone
  // may prefer a stopped rotor over runaway acceleration; neither qualifies
  // as usable. Keep the full sampled coast within explicit retention bounds.
  const minimumSpeedRatio = reference > 0 ? Math.min(...velocities) / reference : null;
  const maximumSpeedRatio = reference > 0 ? Math.max(...velocities) / reference : null;
  const qualificationReasons = [];
  if (stopped || stoppedFraction > 0) qualificationReasons.push('rotor stopped during the measured coast');
  if (reversed) qualificationReasons.push('rotor reversed during the measured coast');
  if (minimumSpeedRatio !== null && minimumSpeedRatio < MIN_COAST_SPEED_RATIO) qualificationReasons.push('coast fell below 50% of pre-release speed');
  if (maximumSpeedRatio !== null && maximumSpeedRatio > MAX_COAST_SPEED_RATIO) qualificationReasons.push('coast exceeded 150% of pre-release speed');
  return {
    valid: reasons.length === 0, reasons, loss: reasons.length ? null : loss,
    qualified: reasons.length === 0 && qualificationReasons.length === 0,
    qualification_reasons: qualificationReasons,
    minimum_speed_ratio: minimumSpeedRatio, maximum_speed_ratio: maximumSpeedRatio,
    target_speed_rad_s: target, velocity_source: source,
    release_start_ms: releaseMs, analyzed_start_ms: firstMs, analyzed_end_ms: lastMs,
    skipped_release_ms: SKIP_RELEASE_MS, sample_count: rows.length, capture_period_ms: expectedPeriod,
    reference_phase: 'pre_release_pi', reference_start_ms: referenceRows[0].ms,
    reference_end_ms: referenceRows.at(-1).ms,
    reference_speed_rad_s: reference * direction, final_speed_rad_s: final * direction,
    post_handover_reference_speed_rad_s: postHandoverReference * direction,
    handover_relative_drift: (postHandoverReference - reference) / norm,
    final_sample_speed_rad_s: rows.at(-1).velocity * direction,
    mean_speed_rad_s: velocityMean * direction,
    slope_rad_s2: slope, signed_axis_slope_rad_s2: slope * direction,
    relative_drift: (final - reference) / norm,
    curve_rmse_relative: Math.sqrt(curveMse), acceleration_rmse_relative: Math.sqrt(accelerationMse),
    velocity_variance_rad2_s2: variance, residual_variance_rad2_s2: residualVariance,
    normalized_variance: normalizedVariance,
    stopped, reversed, stopped_fraction: stoppedFraction, reversed_fraction: reversedFraction,
  };
}

function parseArgs(argv) {
  const flags = new Set(['analyze-only', 'evaluate-only', 'help']);
  const values = new Set(['serial', 'elf', 'out-dir', 'initial-friction-ma', 'initial-viscous-ua',
    'scale', 'cogging-scale', 'inertia-gain', 'inertia-tau-ms', 'speeds', 'duration-ms', 'pulse-ms', 'rounds', 'directions']);
  const args = {};
  for (let i = 0; i < argv.length; ++i) {
    const name = argv[i].replace(/^--/, '');
    if (!argv[i].startsWith('--') || (!flags.has(name) && !values.has(name))) throw new Error(`Unknown option: ${argv[i]}`);
    if (name in args) throw new Error(`Repeated option: --${name}`);
    if (flags.has(name)) args[name] = true;
    else {
      if (argv[i + 1] === undefined || argv[i + 1].startsWith('--')) throw new Error(`Missing value for --${name}`);
      args[name] = argv[++i];
    }
  }
  return args;
}

const HELP = `Usage: node tools/optimize_frictionless.mjs
  --serial <24 hex digits> --elf <flashed diagnostic ELF> --out-dir <results>
  --initial-friction-ma <0..80> --initial-viscous-ua <0..10000>
  [--scale <0..10>] [--cogging-scale <0..2>]
  [--inertia-gain <-1|0..8>] [--inertia-tau-ms <-1|5..200>]
  [--speeds 6,12,20] [--directions positive|negative|both]
  [--duration-ms 6400] [--pulse-ms 1250] [--rounds 2]
  [--evaluate-only] [--analyze-only]

Runs mode 12 only. Fc steps begin at 0.5 mA, B steps at 25 uA/(rad/s),
and halve each round. Scale defaults to 1 and stays fixed; its effect is
redundant with scaling Fc and B. Every candidate uses every speed/direction.
Cogging scale defaults to 1 and stays fixed; use 0 for a correction-disabled A/B run.
Inertia gain/tau default to -1 (use firmware preset), and stay fixed during Fc/B search.
best.json distinguishes qualified best from best_attempt. Qualification requires
every sampled coast to stay in the original direction at 50..150% of its
pre-release speed; it is a usable-candidate screen, not proof of zero friction.
--evaluate-only measures the initial candidate without searching.
--analyze-only evaluates cached captures and never launches a hardware tool.
Interrupts stop scheduling and wait for the active bounded diagnostic to finish.
No settings are saved to the motor, and no firmware is flashed.
`;

export async function main(argv = process.argv.slice(2)) {
  const args = parseArgs(argv);
  if (args.help) { console.log(HELP); return; }
  for (const name of ['serial', 'elf', 'out-dir', 'initial-friction-ma', 'initial-viscous-ua']) {
    if (args[name] === undefined) throw new Error(`Required: --${name}\n${HELP}`);
  }
  const friction = Number(args['initial-friction-ma']);
  const viscous = Number(args['initial-viscous-ua']);
  const scale = Number(args.scale ?? 1);
  const coggingScale = Number(args['cogging-scale'] ?? 1);
  const inertiaGain = Number(args['inertia-gain'] ?? -1);
  const inertiaTauMs = Number(args['inertia-tau-ms'] ?? -1);
  const durationMs = Number(args['duration-ms'] ?? 6400);
  const pulseMs = Number(args['pulse-ms'] ?? 1250);
  const rounds = Number(args.rounds ?? 2);
  const speeds = (args.speeds ?? '6,12,20').split(',').map(Number);
  const directions = args.directions ?? 'both';
  if (!/^[0-9A-Fa-f]{24}$/.test(args.serial) || !Number.isFinite(friction) || friction < 0 || friction > 80
      || !Number.isFinite(viscous) || viscous < 0 || viscous > 10000
      || !Number.isFinite(scale) || scale < 0 || scale > 10
      || !Number.isFinite(coggingScale) || coggingScale < 0 || coggingScale > 2
      || !Number.isFinite(inertiaGain) || (inertiaGain !== -1 && (inertiaGain < 0 || inertiaGain > 8))
      || !Number.isFinite(inertiaTauMs) || (inertiaTauMs !== -1 && (inertiaTauMs < 5 || inertiaTauMs > 200))
      || !Number.isInteger(durationMs) || durationMs < 400 || durationMs > 30000
      || !Number.isInteger(pulseMs) || pulseMs < 1 || pulseMs > 5000 || pulseMs + SETTLING_MS + 800 >= durationMs
      || !Number.isInteger(rounds) || rounds < 0 || rounds > 12
      || !speeds.length || speeds.some(speed => !Number.isFinite(speed) || speed <= 0 || speed > 60)
      || new Set(speeds).size !== speeds.length || !['positive', 'negative', 'both'].includes(directions)) {
    throw new Error('Invalid parameter; no hardware was touched. Use --help for ranges.');
  }
  const elf = resolve(args.elf);
  const outDir = resolve(args['out-dir']);
  const elfHash = createHash('sha256').update(readFileSync(elf)).digest('hex');
  const diagnostic = fileURLToPath(new URL('./motor_diagnostics.mjs', import.meta.url));
  const diagnosticHash = createHash('sha256').update(readFileSync(diagnostic)).digest('hex');
  const config = { analysis_version: ANALYSIS_VERSION, serial: args.serial.toUpperCase(), elf,
    elf_sha256: elfHash, diagnostic_sha256: diagnosticHash, mode: 12,
    duration_ms: durationMs, pulse_ms: pulseMs, speeds, directions, scale, cogging_scale: coggingScale,
    inertia_gain: inertiaGain, inertia_tau_ms: inertiaTauMs,
    coast_qualification: { minimum_speed_ratio: MIN_COAST_SPEED_RATIO, maximum_speed_ratio: MAX_COAST_SPEED_RATIO,
      stop_or_reversal_allowed: false } };
  const fingerprint = createHash('sha256').update(JSON.stringify(config)).digest('hex').slice(0, 16);
  const runDir = join(outDir, fingerprint);
  writeJson(join(runDir, 'config.json'), config);
  const trials = speeds.flatMap(speed => (directions === 'both' ? [speed, -speed] : [directions === 'positive' ? speed : -speed]));
  const evaluated = new Map();
  let stopped = false;
  let activeChild;
  let best;
  let bestAttempt;
  const interrupt = signal => {
    stopped = true;
    console.error(`${signal}: stopping new tests.${activeChild ? ' Waiting for the active timed diagnostic to disable the driver and exit.' : ''}`);
  };
  const onSigint = () => interrupt('SIGINT');
  const onSigterm = () => interrupt('SIGTERM');
  process.on('SIGINT', onSigint);
  process.on('SIGTERM', onSigterm);

  function persist() {
    const results = [...evaluated.values()];
    const summary = { config, fingerprint, interrupted: stopped, analyze_only: !!args['analyze-only'],
      evaluate_only: !!args['evaluate-only'], rounds_requested: rounds,
      best_candidate: best?.parameters ?? null, best_loss: best?.loss ?? null,
      best_attempt_candidate: bestAttempt?.parameters ?? null, best_attempt_loss: bestAttempt?.loss ?? null,
      qualified_result_available: !!best, candidates: results };
    writeJson(join(runDir, 'summary.json'), summary);
    writeJson(join(outDir, 'summary.json'), summary);
    writeJson(join(outDir, 'best.json'), { config, fingerprint, interrupted: stopped,
      note: 'best meets all declared coast qualification bounds; best_attempt may fail them. No parameters were saved to hardware.',
      best: best ?? null, best_attempt: bestAttempt ?? null });
  }

  async function runDiagnostic(path, parameters, speed) {
    const childArgs = [diagnostic, '--serial', config.serial, '--elf', elf, '--out', path,
      '--mode', '12', '--current-ma', '0', '--speed-rad-s', String(speed),
      '--pulse-ms', String(pulseMs), '--duration-ms', String(durationMs),
      '--friction-ma', String(parameters.friction_ma), '--viscous-ua', String(parameters.viscous_ua), '--scale', String(scale),
      '--cogging-scale', String(coggingScale), '--inertia-gain', String(inertiaGain), '--inertia-tau-ms', String(inertiaTauMs)];
    mkdirSync(dirname(path), { recursive: true });
    // A separate process group keeps Ctrl+C from interrupting the child before
    // its mailbox abort/release and OpenOCD shutdown finally block can run.
    const child = spawn(process.execPath, childArgs, { stdio: ['ignore', 'pipe', 'pipe'], windowsHide: true, detached: true });
    activeChild = child;
    let stdout = '';
    let stderr = '';
    child.stdout.on('data', data => { stdout += data; });
    child.stderr.on('data', data => { stderr += data; });
    const result = await new Promise(resolveResult => {
      child.once('error', error => resolveResult({ exit_code: null, error: error.message }));
      child.once('close', (code, signal) => resolveResult({ exit_code: code, signal }));
    });
    activeChild = undefined;
    writeJson(path.replace(/\.json$/, '.driver.json'), { ...result, stdout, stderr });
    return result;
  }

  async function evaluate(parameters) {
    const candidateKey = `fc${parameters.friction_ma}_b${parameters.viscous_ua}_s${scale}_c${coggingScale}`;
    if (evaluated.has(candidateKey)) return evaluated.get(candidateKey);
    const candidateDir = join(runDir, candidateKey);
    const result = { parameters: { ...parameters, scale, cogging_scale: coggingScale, inertia_gain: inertiaGain, inertia_tau_ms: inertiaTauMs },
      valid: false, qualified: false, loss: null, tests: [] };
    console.log(`Candidate Fc=${parameters.friction_ma} mA B=${parameters.viscous_ua} uA/(rad/s) scale=${scale} cogging=${coggingScale} inertia=${inertiaGain}/${inertiaTauMs}ms`);
    for (const speed of trials) {
      if (stopped) break;
      const path = join(candidateDir, `${speed > 0 ? 'positive' : 'negative'}-${Math.abs(speed)}.json`);
      let driver;
      const cached = existsSync(path);
      const driverPath = path.replace(/\.json$/, '.driver.json');
      if (cached && existsSync(driverPath)) {
        try { driver = readJson(driverPath); }
        catch (error) { driver = { exit_code: null, error: `invalid cached driver result: ${error.message}` }; }
      }
      if (!cached && !args['analyze-only']) driver = await runDiagnostic(path, parameters, speed);
      let analysis;
      if (existsSync(path)) {
        try {
          const capture = readJson(path);
          const expected = { mode: 12, friction_ma: parameters.friction_ma, viscous_microamps: parameters.viscous_ua,
            compensation_scale: scale, cogging_scale: coggingScale,
            inertia_gain: inertiaGain, inertia_tau_ms: inertiaTauMs,
            pulse_velocity_rad_s: speed, pulse_ms: pulseMs, duration_ms: durationMs, current_ma: 0 };
          const mismatch = Object.entries(expected).filter(([key, value]) => !Number.isFinite(capture.options?.[key])
            || Math.abs(capture.options[key] - value) > 1e-6).map(([key]) => key);
          analysis = analyzeRelease(capture, { speed_rad_s: speed, pulse_ms: pulseMs, duration_ms: durationMs });
          if (mismatch.length) analysis.reasons.push(`capture parameters mismatch: ${mismatch.join(', ')}`);
          if (driver && driver.exit_code !== 0) analysis.reasons.push(`driver exit ${driver.exit_code}: ${driver.error ?? driver.signal ?? 'failure'}`);
          analysis.valid = analysis.reasons.length === 0;
          if (!analysis.valid) { analysis.loss = null; analysis.qualified = false; }
        } catch (error) {
          analysis = { valid: false, qualified: false, loss: null, reasons: [`cannot analyze capture: ${error.message}`] };
        }
      } else {
        analysis = { valid: false, qualified: false, loss: null, reasons: [args['analyze-only'] ? 'capture not cached'
          : `driver produced no capture (exit ${driver?.exit_code}): ${driver?.error ?? driver?.signal ?? 'failure'}`] };
      }
      result.tests.push({ speed_rad_s: speed, capture: path, cached, ...analysis });
      console.log(JSON.stringify({ speed_rad_s: speed, cached, valid: analysis.valid, qualified: analysis.qualified, loss: analysis.loss,
        slope_rad_s2: analysis.slope_rad_s2, relative_drift: analysis.relative_drift,
        final_speed_rad_s: analysis.final_speed_rad_s, stopped: analysis.stopped, reversed: analysis.reversed,
        reasons: analysis.reasons, qualification_reasons: analysis.qualification_reasons }));
      writeJson(join(candidateDir, 'analysis.json'), result);
    }
    result.valid = result.tests.length === trials.length && result.tests.every(test => test.valid);
    result.qualified = result.valid && result.tests.every(test => test.qualified);
    if (result.valid) {
      // Equal weight per speed and direction; a poor single direction remains
      // visible instead of being hidden by excellent performance elsewhere.
      const losses = result.tests.map(test => test.loss);
      result.mean_loss = mean(losses);
      result.worst_loss = Math.max(...losses);
      result.loss = result.mean_loss + 0.25 * result.worst_loss;
      if (!bestAttempt || result.loss < bestAttempt.loss) bestAttempt = result;
      if (result.qualified && (!best || result.loss < best.loss)) best = result;
    }
    evaluated.set(candidateKey, result);
    writeJson(join(candidateDir, 'analysis.json'), result);
    persist();
    return result;
  }

  try {
    let center = { friction_ma: round(friction), viscous_ua: round(viscous) };
    let centerResult = await evaluate(center);
    if (!args['evaluate-only']) {
      for (let roundIndex = 0; roundIndex < rounds && !stopped; ++roundIndex) {
        for (const [key, initialStep, max] of [['friction_ma', 0.5, 80], ['viscous_ua', 25, 10000]]) {
          if (stopped) break;
          const step = initialStep / (2 ** roundIndex);
          const candidates = [centerResult];
          for (const sign of [-1, 1]) {
            if (stopped) break;
            const value = round(center[key] + sign * step);
            if (value >= 0 && value <= max) candidates.push(await evaluate({ ...center, [key]: value }));
          }
          const winner = candidates.filter(candidate => candidate.valid)
            .sort((a, b) => Number(b.qualified) - Number(a.qualified) || a.loss - b.loss)[0];
          if (winner) {
            center = { friction_ma: winner.parameters.friction_ma, viscous_ua: winner.parameters.viscous_ua };
            centerResult = winner;
          }
        }
      }
    }
    persist();
    console.log(JSON.stringify({ best: best?.parameters ?? null, loss: best?.loss ?? null,
      best_attempt: bestAttempt?.parameters ?? null, best_attempt_loss: bestAttempt?.loss ?? null,
      qualified_result_available: !!best,
      candidates: evaluated.size, interrupted: stopped, summary: join(outDir, 'summary.json') }));
    if (stopped) process.exitCode = 130;
    else if (!bestAttempt) process.exitCode = 1;
  } finally {
    process.removeListener('SIGINT', onSigint);
    process.removeListener('SIGTERM', onSigterm);
  }
}

if (process.argv[1] && pathToFileURL(resolve(process.argv[1])).href === import.meta.url) {
  main().catch(error => { console.error(error.stack ?? error); process.exitCode = 1; });
}
