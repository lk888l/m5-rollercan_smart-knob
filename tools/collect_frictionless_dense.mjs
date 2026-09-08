// Dense angle-coverage collection only. Default is an offline plan; --execute
// runs bounded diagnostics serially. Never flashes or saves motor settings.
import { spawn } from 'node:child_process';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, readdirSync, statSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { prepareDenseCapture } from './identify_frictionless_dense.mjs';

const TWO_PI = 2 * Math.PI;
const BINS = 32;
const DRIVER = fileURLToPath(new URL('./motor_diagnostics.mjs', import.meta.url));
const mean = values => values.reduce((sum, value) => sum + value, 0) / values.length;
const binAt = angle => Math.floor(((angle % TWO_PI) + TWO_PI) % TWO_PI / TWO_PI * BINS);
const encode = value => JSON.stringify(value, null, 2) + '\n';

export function predictedCaptureBins(endAngle, speed) {
  // 383 PI samples minus 50 ms filter warmup and averaging tails leave at
  // least 320 ms. Use a conservative arc to tolerate phase prediction error.
  const bins = new Set();
  for (let ms = 0; ms < 320; ++ms) bins.add(binAt(endAngle - speed * ms / 1000));
  return [...bins];
}

export function choosePulse(angle, speed, support, minimumSupport = 2, lagSeconds = 0.15) {
  let best;
  // Change pulse length while continuing in the same direction. Paired +v/-v
  // pulses cancel net displacement and repeatedly return to the same arc.
  for (let pulseMs = 2500; pulseMs <= 5000; pulseMs += 10) {
    const endAngle = angle + speed * (pulseMs / 1000 - lagSeconds);
    const bins = predictedCaptureBins(endAngle, speed);
    const score = bins.reduce((sum, bin) => sum + (support[bin] < minimumSupport ? minimumSupport - support[bin] : 0), 0);
    if (!best || score > best.score) best = { pulse_ms: pulseMs, predicted_end_angle: endAngle, predicted_bins: bins, score };
  }
  return best;
}

function jsonFiles(directory) {
  if (!existsSync(directory)) return [];
  return readdirSync(directory).flatMap(name => {
    const path = join(directory, name);
    return statSync(path).isDirectory() ? jsonFiles(path) : name.endsWith('.json') ? [path] : [];
  });
}

function measuredSupport(paths, serial, direction) {
  const bins = Array(BINS).fill(0), accepted = [];
  for (const path of paths) {
    let capture;
    try { capture = JSON.parse(readFileSync(path, 'utf8')); } catch { continue; }
    if (capture.options?.serial?.toUpperCase() !== serial || Math.sign(capture.options?.pulse_velocity_rad_s) !== direction) continue;
    const prepared = prepareDenseCapture(capture, path, [7, 12]);
    if (!prepared.rows.length || capture.metrics?.app_control_deadline_miss_count !== 0) continue;
    prepared.angle_bin_samples.forEach((count, bin) => { if (count > 0) ++bins[bin]; });
    accepted.push(path);
  }
  return { bins, accepted };
}

const HELP = `node tools/collect_frictionless_dense.mjs
  --serial <24 hex digits> --elf <flashed ABI5 ELF> --out-dir <new captures>
  [--existing-dir <prior dense captures>] [--direction negative|positive]
  [--speed 6] [--min-support 2] [--max-captures 10] [--start-angle 0]
  [--execute]

Default: print an OFFLINE phase-coverage plan, without starting any hardware tool.
--execute: run same-direction speed PI captures, varying pulse length 2500..5000
ms to fill the least-supported mechanical-angle bins. Recompute actual 32-bin
coverage after every capture. Stop only when each bin has the requested number
of independent captures, or report incomplete coverage at the capture limit.
Each test uses --mode off --dense 1, duration=pulse+351, leaving one release ms.
An unknown initial shaft angle is acceptable: the first capture establishes it.
The initial phase argument improves only the first prediction. Existing captures
must be from the same calibrated module and comparable firmware/current sensing.
No firmware is flashed and no persistent mode parameters are written.
`;

export async function main(argv = process.argv.slice(2)) {
  const args = {}, flags = new Set(['execute', 'help']);
  const values = new Set(['serial', 'elf', 'out-dir', 'existing-dir', 'direction', 'speed', 'min-support', 'max-captures', 'start-angle']);
  for (let i = 0; i < argv.length; ++i) {
    const name = argv[i].replace(/^--/, '');
    if (!argv[i].startsWith('--') || (!flags.has(name) && !values.has(name))) throw new Error(`Unknown argument ${argv[i]}`);
    if (name in args) throw new Error(`Repeated argument --${name}`);
    if (flags.has(name)) args[name] = true;
    else if (argv[i + 1] === undefined || argv[i + 1].startsWith('--')) throw new Error(`Missing value for --${name}`);
    else args[name] = argv[++i];
  }
  if (args.help) { console.log(HELP); return; }
  if (!/^[0-9a-f]{24}$/i.test(args.serial ?? '') || !args.elf || !args['out-dir']) throw new Error(HELP);
  const serial = args.serial.toUpperCase(), elf = resolve(args.elf), outDir = resolve(args['out-dir']);
  const directionName = args.direction ?? 'negative';
  const magnitude = Number(args.speed ?? 6), minimumSupport = Number(args['min-support'] ?? 2);
  const maximumCaptures = Number(args['max-captures'] ?? 10);
  let angle = Number(args['start-angle'] ?? 0);
  if (!['negative', 'positive'].includes(directionName) || !Number.isFinite(magnitude) || magnitude < 1 || magnitude > 12
      || !Number.isInteger(minimumSupport) || minimumSupport < 2 || minimumSupport > 10
      || !Number.isInteger(maximumCaptures) || maximumCaptures < 1 || maximumCaptures > 60 || !Number.isFinite(angle)) {
    throw new Error('Invalid collection parameter; no hardware was touched');
  }
  const direction = directionName === 'negative' ? -1 : 1, speed = direction * magnitude;
  const elfHash = createHash('sha256').update(readFileSync(elf)).digest('hex');
  const baselinePaths = args['existing-dir'] ? jsonFiles(resolve(args['existing-dir'])) : [];
  const outputPaths = jsonFiles(outDir);
  const paths = [...new Set([...baselinePaths, ...outputPaths])];
  let support = measuredSupport(paths, serial, direction);
  const manifest = { serial, elf, elf_sha256: elfHash, execution_requested: !!args.execute,
    direction: directionName, speed_rad_s: speed, minimum_capture_support_per_bin: minimumSupport,
    initial_support: support.bins, accepted_prior_captures: support.accepted, trials: [] };
  let lagSeconds = 0.15, previousEnd, stopped = false, child;
  const interrupt = () => {
    stopped = true;
    console.error(`Interrupt received; no further captures will start.${child ? ' Waiting for the current bounded diagnostic to disable output and exit.' : ''}`);
  };
  process.on('SIGINT', interrupt);
  process.on('SIGTERM', interrupt);
  const persist = () => {
    if (!args.execute) return;
    mkdirSync(outDir, { recursive: true });
    writeFileSync(join(outDir, 'collection.json'), encode({ ...manifest, interrupted: stopped,
      completed: support.bins.every(count => count >= minimumSupport), final_support: support.bins }));
  };
  try {
    for (let index = 0; index < maximumCaptures && !stopped && support.bins.some(count => count < minimumSupport); ++index) {
      const plan = choosePulse(angle, speed, support.bins, minimumSupport, lagSeconds);
      let id = index + 1;
      while (existsSync(join(outDir, `capture-${directionName}-${String(id).padStart(3, '0')}.json`))) ++id;
      const path = join(outDir, `capture-${directionName}-${String(id).padStart(3, '0')}.json`);
      const childArgs = [DRIVER, '--serial', serial, '--elf', elf, '--out', path, '--mode', 'off', '--dense', '1',
        '--current-ma', '0', '--speed-rad-s', String(speed), '--pulse-ms', String(plan.pulse_ms),
        '--duration-ms', String(plan.pulse_ms + 351)];
      const trial = { ...plan, capture: path, executable: process.execPath, args: childArgs };
      manifest.trials.push(trial);
      console.log(encode({ action: args.execute ? 'collect' : 'offline_plan', ...trial }).trim());
      if (!args.execute) {
        for (const bin of plan.predicted_bins) ++support.bins[bin];
        angle = plan.predicted_end_angle;
        continue;
      }
      mkdirSync(outDir, { recursive: true });
      child = spawn(process.execPath, childArgs, { windowsHide: true, detached: true, stdio: ['ignore', 'pipe', 'pipe'] });
      let stdout = '', stderr = '';
      child.stdout.on('data', data => { stdout += data; });
      child.stderr.on('data', data => { stderr += data; });
      const result = await new Promise(resolveResult => {
        child.once('error', error => resolveResult({ exit_code: null, error: error.message }));
        child.once('close', (code, signal) => resolveResult({ exit_code: code, signal }));
      });
      child = undefined;
      trial.driver = { ...result, stdout, stderr };
      if (result.exit_code !== 0 || !existsSync(path)) { persist(); throw new Error(`Dense diagnostic failed: ${stderr || result.error || result.exit_code}`); }
      const capture = JSON.parse(readFileSync(path, 'utf8'));
      const prepared = prepareDenseCapture(capture, path, [7, 12]);
      if (!prepared.rows.length || capture.metrics?.app_control_deadline_miss_count !== 0) {
        persist(); throw new Error(`No valid moving PI intervals: ${prepared.reasons.join(', ')}`);
      }
      const tail = capture.samples.filter(row => row.ms < 350 + plan.pulse_ms).slice(-50);
      trial.mean_final_pi_velocity_rad_s = mean(tail.map(row => row.velocity));
      const end = tail.at(-1).angle;
      if (previousEnd !== undefined) {
        const measuredLag = plan.pulse_ms / 1000 - (end - previousEnd) / speed;
        if (measuredLag >= 0 && measuredLag <= 0.5) lagSeconds = measuredLag;
      }
      previousEnd = end;
      angle = end;
      paths.push(path);
      support = measuredSupport(paths, serial, direction);
      trial.actual_supported_bins = prepared.angle_bin_samples.flatMap((count, bin) => count ? [bin] : []);
      trial.remaining_bins = support.bins.flatMap((count, bin) => count < minimumSupport ? [bin] : []);
      trial.phase_prediction_error_rad = Math.atan2(Math.sin(end - plan.predicted_end_angle), Math.cos(end - plan.predicted_end_angle));
      persist();
      console.log(encode({ capture: path, actual_supported_bins: trial.actual_supported_bins,
        remaining_bins: trial.remaining_bins, minimum_support: Math.min(...support.bins) }).trim());
    }
    persist();
    const complete = support.bins.every(count => count >= minimumSupport);
    console.log(encode({ completed: args.execute ? complete : null, predicted_complete: args.execute ? undefined : complete,
      note: args.execute ? 'Measured coverage only; no settings were saved.' : 'Predicted coverage is not a motor test or proof of actual angle support.',
      captures: manifest.trials.length, support: support.bins, interrupted: stopped }).trim());
    if (stopped) process.exitCode = 130;
    else if (!complete) process.exitCode = 1;
  } finally {
    process.removeListener('SIGINT', interrupt);
    process.removeListener('SIGTERM', interrupt);
  }
}

if (process.argv[1] && pathToFileURL(resolve(process.argv[1])).href === import.meta.url) {
  main().catch(error => { console.error(error.stack ?? error); process.exitCode = 1; });
}
