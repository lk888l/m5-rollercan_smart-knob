// Node.js + OpenOCD + arm-none-eabi-nm; no packages required.
// Uses live SWD memory accesses, so ControlTask and the FOC ISR keep running.
import net from 'node:net';
import { spawn, execFileSync } from 'node:child_process';
import { writeFileSync, mkdirSync } from 'node:fs';
import { dirname } from 'node:path';
import { setTimeout as delay } from 'node:timers/promises';

const args = Object.fromEntries(process.argv.slice(2).reduce((pairs, value, i, all) => {
  if (i % 2 === 0) pairs.push([value.replace(/^--/, ''), all[i + 1]]);
  return pairs;
}, []));
if (!/^[0-9A-Fa-f]{24}$/.test(args.serial ?? '') || !args.elf || !args.out) {
  throw new Error('Required: --serial <24 hex digits> --elf <flashed ELF> --out <JSON> [--mode coast|12] [--current-ma 100] [--pulse-ms 500] [--duration-ms 4000] [--dense 0|1]');
}
const nm = execFileSync('arm-none-eabi-nm', ['--defined-only', args.elf], { encoding: 'utf8', windowsHide: true });
const symbols = new Map(nm.trim().split(/\r?\n/).map(line => {
  const [address, , name] = line.trim().split(/\s+/);
  return [name, parseInt(address, 16)];
}));
const address = symbols.get('motor_debug');
if (!address) throw new Error('ELF must be built with ROLLERCAN_MOTOR_DIAGNOSTICS=ON');
const options = {
  serial: args.serial, mode: ({ coast: 0xffffffff, 'torque-zero': 0xfffffffe, off: 0xfffffffd })[args.mode ?? 'coast'] ?? Number(args.mode),
  pulse_ms: Number(args['pulse-ms'] ?? 500), duration_ms: Number(args['duration-ms'] ?? 4000),
  current_ma: Number(args['current-ma'] ?? 100),
  friction_ma: Number(args['friction-ma'] ?? -1),
  viscous_microamps: Number(args['viscous-ua'] ?? -1),
  pulse_velocity_rad_s: Number(args['speed-rad-s'] ?? 0),
  compensation_scale: Number(args.scale ?? -1),
  cogging_scale: Number(args['cogging-scale'] ?? 1),
  dense: Number(args.dense ?? 0),
  inertia_gain: Number(args['inertia-gain'] ?? -1),
  inertia_tau_ms: Number(args['inertia-tau-ms'] ?? -1),
};
const saveArgument = args['save-mode'] ?? args['save-mode-limits'];
const saveMode = saveArgument === undefined ? undefined : Number(saveArgument);
if (!Number.isInteger(options.mode) || !(options.mode <= 12 || options.mode >= 0xfffffffd) ||
    options.mode < 0 || options.mode > 0xffffffff ||
    !Number.isInteger(options.pulse_ms) || options.pulse_ms < 0 || options.pulse_ms > 5000 ||
    !Number.isInteger(options.duration_ms) || options.duration_ms < 400 || options.duration_ms > 30000 ||
    options.pulse_ms + 350 >= options.duration_ms ||
    !Number.isFinite(options.current_ma) || Math.abs(options.current_ma) > 250 ||
    !Number.isFinite(options.pulse_velocity_rad_s) || Math.abs(options.pulse_velocity_rad_s) > 60 ||
    !Number.isInteger(options.dense) || options.dense < 0 || options.dense > 1 ||
    !Number.isFinite(options.inertia_gain) || (options.inertia_gain !== -1 && (options.inertia_gain < 0 || options.inertia_gain > 8)) ||
    !Number.isFinite(options.inertia_tau_ms) || (options.inertia_tau_ms !== -1 && (options.inertia_tau_ms < 5 || options.inertia_tau_ms > 200)) ||
    (options.dense === 1 && (options.pulse_velocity_rad_s === 0 || options.pulse_ms === 0)) ||
    !Number.isFinite(options.compensation_scale) || options.compensation_scale < -1 || options.compensation_scale > 10 ||
    !Number.isFinite(options.cogging_scale) || options.cogging_scale < 0 || options.cogging_scale > 2 ||
    !Number.isFinite(options.friction_ma) || options.friction_ma < -1 || options.friction_ma > 80 ||
    !Number.isFinite(options.viscous_microamps) || options.viscous_microamps < -1 || options.viscous_microamps > 10000 ||
    (saveMode !== undefined && (!Number.isInteger(saveMode) || saveMode < 0 || saveMode > 12)) ||
    (args['save-mode'] !== undefined && args['save-mode-limits'] !== undefined)) {
  throw new Error('Invalid mode, duration or motor-test parameter; no hardware was touched');
}
const server = spawn('openocd', [
  '-f', 'interface/stlink.cfg', '-c', `adapter serial ${args.serial}`,
  '-c', 'transport select swd', '-f', 'target/stm32g4x.cfg',
  '-c', 'adapter speed 1800', '-c', 'gdb port disabled', '-c', 'telnet port disabled',
  '-c', 'tcl port 6667', '-c', 'init',
], { windowsHide: true, stdio: ['ignore', 'pipe', 'pipe'] });
let log = '';
server.stdout.on('data', data => { log += data; });
server.stderr.on('data', data => { log += data; });
let socket;
let pending;
let received = '';
let mailboxVerified = false;
let requestSubmitted = false;
function command(text) {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => { pending = undefined; reject(new Error(`OpenOCD timeout: ${text}`)); }, 5000);
    pending = { resolve: value => { clearTimeout(timer); resolve(value); }, reject };
    socket.write(text + '\x1a');
  });
}
async function readWords(at, count) {
  if (count === 0) return [];
  if (count > 64) {
    const result = [];
    for (let offset = 0; offset < count; offset += 64) {
      result.push(...await readWords(at + offset * 4, Math.min(64, count - offset)));
    }
    return result;
  }
  for (let attempt = 0; attempt < 3; ++attempt) {
    // Some V2 probes fail long USB memory transfers. Single-word SWD reads
    // inside one Tcl request also work for their completed capture buffers.
    const addresses = Array.from({ length: count }, (_, i) => '0x' + (at + i * 4).toString(16)).join(' ');
    const response = await command(`set motor_values {}; foreach motor_addr {${addresses}} {lappend motor_values [read_memory $motor_addr 32 1]}; return $motor_values`);
    const words = response.trim().split(/\s+/).map(Number);
    if (words.length === count && words.every(value => Number.isFinite(value))) return words;
    if (attempt === 2) throw new Error(`Unexpected memory read at 0x${at.toString(16)}, count ${count}: ${response}`);
    await delay(100);
  }
}
async function writeWords(at, values) {
  const response = await command(`write_memory 0x${at.toString(16)} 32 {${values.map(value => '0x' + (value >>> 0).toString(16)).join(' ')}}`);
  if (/error|invalid|failed/i.test(response)) throw new Error(response);
}
function floatWord(value) {
  const buffer = Buffer.alloc(4); buffer.writeFloatLE(value); return buffer.readUInt32LE();
}
try {
  // Never attach to an unrelated server already occupying our TCP port.
  for (let attempt = 0; attempt < 50 && !log.includes('Listening on port 6667'); ++attempt) {
    if (server.exitCode !== null) break;
    await delay(100);
  }
  if (!log.includes('Listening on port 6667') || server.exitCode !== null) {
    throw new Error(`Our OpenOCD instance did not start\n${log}`);
  }
  for (let attempt = 0; attempt < 30; ++attempt) {
    try {
      socket = await new Promise((resolve, reject) => {
        const candidate = net.connect(6667, '127.0.0.1');
        candidate.once('connect', () => resolve(candidate));
        candidate.once('error', reject);
      });
      break;
    } catch { await delay(100); }
  }
  if (!socket) throw new Error(`Cannot connect to OpenOCD\n${log}`);
  socket.on('data', data => {
    received += data.toString();
    const end = received.indexOf('\x1a');
    if (end >= 0) {
      const response = received.slice(0, end); received = received.slice(end + 1);
      const handler = pending; pending = undefined; handler?.resolve(response);
    }
  });
  if ((await readWords(address, 1))[0] !== 0x4b4e4406) throw new Error('Flashed firmware does not match diagnostic ABI v6');
  mailboxVerified = true;
  if ([1, 5].includes((await readWords(address + 8, 1))[0])) {
    throw new Error('A diagnostic request is already running');
  }
  await writeWords(address + 12, [options.mode, options.pulse_ms, options.duration_ms, floatWord(options.current_ma)]);
  await writeWords(address + 36, [floatWord(options.friction_ma), floatWord(options.viscous_microamps), floatWord(options.pulse_velocity_rad_s), floatWord(options.compensation_scale), floatWord(options.cogging_scale)]);
  await writeWords(address + 56, [options.dense]);
  await writeWords(address + 60, [floatWord(options.inertia_gain), floatWord(options.inertia_tau_ms)]);
  const action = saveMode === undefined ? 1 : args['save-mode-limits'] !== undefined ? 5 : 4;
  if (saveMode !== undefined) await writeWords(address + 12, [saveMode]);
  requestSubmitted = true;
  await writeWords(address + 4, [action]);
  const deadline = Date.now() + options.duration_ms + 5000;
  let header;
  do {
    await delay(100);
    header = await readWords(address, 9);
    if ([2, 3, 4].includes(header[2])) break;
  } while (Date.now() < deadline);
  if (![2, 3, 4].includes(header[2])) throw new Error('ControlTask did not finish the request');
  // The test has already disabled the bridge and frozen its capture buffer.
  const sampleFields = options.dense === 1
    ? ['angle', 'velocity', 'filtered_command_ma']
    : ['angle', 'velocity', 'slow_velocity', 'command_ma', 'measured_ma', 'uq_v', 'mean_command_ma', 'mean_velocity_rad_s', 'mean_filtered_command_ma', 'mean_filtered_sin12', 'mean_filtered_cos12'];
  const sampleWords = sampleFields.length + 1;
  const count = Math.min(header[7], options.dense === 1 ? 384 : 128);
  const words = await readWords(address + 68, count * sampleWords);
  const bytes = Buffer.alloc(words.length * 4);
  words.forEach((word, index) => bytes.writeUInt32LE(word, index * 4));
  const samples = Array.from({ length: count }, (_, index) => {
    const base = index * sampleWords * 4;
    const row = { ms: bytes.readUInt32LE(base) };
    sampleFields.forEach((key, offset) => {
      row[key] = bytes.readFloatLE(base + 4 + offset * 4);
    });
    return row;
  });
  const metrics = {};
  for (const name of ['app_control_step_count', 'app_control_deadline_miss_count', 'app_control_stack_min_words', 'app_maintenance_stack_min_words', 'app_storage_stack_min_words', 'runtime_control_max_cycles', 'SystemCoreClock']) {
    if (symbols.has(name)) metrics[name] = (await readWords(symbols.get(name), 1))[0];
  }
  mkdirSync(dirname(args.out), { recursive: true });
  writeFileSync(args.out, JSON.stringify({ options, status: header[2], metrics, samples }, null, 2));
  writeFileSync(args.out.replace(/\.json$/, '') + '.openocd.log', log);
  console.log(JSON.stringify({ out: args.out, status: header[2], samples: count, metrics,
    peak_speed: Math.max(...samples.map(row => Math.abs(row.velocity))),
    final_speed: samples.at(-1)?.velocity,
    travel_rad: samples.at(-1)?.angle - samples[0]?.angle,
  }));
  if (header[2] !== 2) process.exitCode = 1;
} catch (error) {
  mkdirSync(dirname(args.out), { recursive: true });
  writeFileSync(args.out.replace(/\.json$/, '') + '.openocd.log', log);
  throw error;
} finally {
  if (socket && !socket.destroyed) {
    if (mailboxVerified && requestSubmitted && saveMode === undefined) {
      await writeWords(address + 4, [2]).catch(() => {});
      await delay(30);
      await writeWords(address + 4, [3]).catch(() => {});
      await delay(30);
    }
    await command('shutdown').catch(() => {});
    socket.destroy();
  }
  if (server.exitCode === null) {
    await Promise.race([new Promise(resolve => server.once('exit', resolve)), delay(1000)]);
    if (server.exitCode === null) server.kill();
  }
  await delay(300);
}
