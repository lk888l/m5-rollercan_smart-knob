// Sequential loss identification through the existing, bounded motor diagnostic.
// No settings are saved and no firmware is flashed by this script.
import { spawn } from 'node:child_process';
import { existsSync, mkdirSync, readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { resolve, join } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';

const WINDOW_START_MS = 3350;
const WINDOW_END_MS = 5350;
const mean = values => values.reduce((sum, value) => sum + value, 0) / values.length;
const weightedMean = (values, weights) => values.reduce((sum, value, index) => sum + value * weights[index], 0) / weights.reduce((sum, weight) => sum + weight, 0);
const weightedDeviation = (values, weights) => {
  const average = weightedMean(values, weights);
  const weightSum = weights.reduce((sum, weight) => sum + weight, 0);
  const denominator = weightSum - weights.reduce((sum, weight) => sum + weight ** 2, 0) / weightSum;
  return Math.sqrt(values.reduce((sum, value, index) => sum + weights[index] * (value - average) ** 2, 0) / denominator);
};
const deviation = values => {
  const average = mean(values);
  return Math.sqrt(values.reduce((sum, value) => sum + (value - average) ** 2, 0) / Math.max(1, values.length - 1));
};

function linearTrend(times, values, weights) {
  const timeMean = weightedMean(times, weights);
  const valueMean = weightedMean(values, weights);
  const denominator = times.reduce((sum, time, index) => sum + weights[index] * (time - timeMean) ** 2, 0);
  const slope = times.reduce((sum, time, index) => sum + weights[index] * (time - timeMean) * (values[index] - valueMean), 0) / denominator;
  const residuals = times.map((time, index) => values[index] - valueMean - slope * (time - timeMean));
  return { slope, residual_rms: Math.sqrt(weightedMean(residuals.map(value => value ** 2), weights)) };
}

export function analyzeCapture(capture, source) {
  const target = capture.options?.pulse_velocity_rad_s;
  const allRows = capture.samples ?? [];
  const rows = allRows.map((row, index) => ({
    ...row,
    interval_ms: row.ms - (allRows[index - 1]?.ms ?? 0),
    fit_velocity: Number.isFinite(row.mean_velocity_rad_s) ? row.mean_velocity_rad_s : row.velocity,
    fit_command: Number.isFinite(row.mean_command_ma) ? row.mean_command_ma : row.command_ma,
  })).filter(row => row.ms >= WINDOW_START_MS && row.ms < WINDOW_END_MS);
  const reasons = [];
  if (capture.status !== 2) reasons.push(`诊断未正常完成（状态 ${capture.status}）`);
  if (capture.metrics?.app_control_deadline_miss_count > 0) reasons.push(`控制周期超时计数为 ${capture.metrics.app_control_deadline_miss_count}`);
  if (!Number.isFinite(target) || target === 0) reasons.push('目标速度无效');
  if (capture.options?.mode !== 0xfffffffd || capture.options?.pulse_ms !== 5000) {
    reasons.push('不是预期的 off 模式 5000 ms 速度 PI 测试');
  }
  if (rows.length < 20 || rows.some(row => row.interval_ms <= 0 || !['ms', 'angle', 'fit_velocity', 'fit_command', 'interval_ms'].every(key => Number.isFinite(row[key])))) {
    return { source, target_velocity_rad_s: target, samples: rows.length, steady: false, reasons: [...reasons, '稳态窗口有效样本不足'] };
  }
  const times = rows.map(row => row.ms / 1000);
  const weights = rows.map(row => row.interval_ms);
  const velocities = rows.map(row => row.fit_velocity);
  const currents = rows.map(row => row.fit_command);
  const velocity = weightedMean(velocities, weights);
  const current = weightedMean(currents, weights);
  const velocityTrend = linearTrend(rows.map(row => (row.ms - (Number.isFinite(row.mean_velocity_rad_s) ? row.interval_ms / 2 : 0)) / 1000), velocities, weights);
  const currentTrend = linearTrend(rows.map(row => (row.ms - (Number.isFinite(row.mean_command_ma) ? row.interval_ms / 2 : 0)) / 1000), currents, weights);
  const angleTrend = linearTrend(times, rows.map(row => row.angle), weights);
  const speedTolerance = Math.max(0.35, Math.abs(target) * 0.08);
  const accelerationTolerance = Math.max(0.2, Math.abs(target) * 0.03);
  const rippleTolerance = Math.max(0.5, Math.abs(target) * 0.1);
  if (times.at(-1) - times[0] < 1.5) reasons.push('窗口覆盖不足 1.5 秒');
  if (Math.sign(velocity) !== Math.sign(target) || Math.abs(velocity - target) > speedTolerance) reasons.push('平均速度未跟随目标');
  if (Math.abs(velocityTrend.slope) > accelerationTolerance) reasons.push('仍有明显加速或减速');
  if (weightedDeviation(velocities, weights) > rippleTolerance) reasons.push('速度波动过大');
  if (Math.abs(angleTrend.slope - velocity) > speedTolerance) reasons.push('角度斜率与速度不一致');
  if (Math.abs(currentTrend.slope) > Math.max(1, Math.abs(current) * 0.1)) reasons.push('电流仍有明显变化趋势');
  return {
    source, target_velocity_rad_s: target, samples: rows.length,
    window_start_ms: rows[0].ms, window_end_ms: rows.at(-1).ms,
    velocity_rad_s: velocity, velocity_std_rad_s: weightedDeviation(velocities, weights),
    acceleration_rad_s2: velocityTrend.slope, angle_velocity_rad_s: angleTrend.slope,
    angle_linearity_rms_rad: angleTrend.residual_rms,
    command_ma: current, command_std_ma: weightedDeviation(currents, weights),
    command_trend_ma_s: currentTrend.slope,
    weighted_duration_ms: weights.reduce((sum, weight) => sum + weight, 0),
    averaged_command_samples: rows.filter(row => Number.isFinite(row.mean_command_ma)).length,
    averaged_velocity_samples: rows.filter(row => Number.isFinite(row.mean_velocity_rad_s)).length,
    // Fit the steady command mean; measured_ma is noisier instantaneous Iq feedback.
    steady: reasons.length === 0, reasons,
    qualification_limits: {
      target_error_rad_s: speedTolerance, acceleration_rad_s2: accelerationTolerance,
      velocity_std_rad_s: rippleTolerance,
    },
    diagnostic_metrics: capture.metrics ?? {},
  };
}

function solve(matrix, vector) {
  const augmented = matrix.map((row, index) => [...row, vector[index]]);
  for (let column = 0; column < vector.length; ++column) {
    let pivot = column;
    for (let row = column + 1; row < vector.length; ++row) {
      if (Math.abs(augmented[row][column]) > Math.abs(augmented[pivot][column])) pivot = row;
    }
    if (Math.abs(augmented[pivot][column]) < 1e-10) throw new Error('速度点不足以分离库仑摩擦、黏性损耗与偏置');
    [augmented[column], augmented[pivot]] = [augmented[pivot], augmented[column]];
    const divisor = augmented[column][column];
    for (let item = column; item <= vector.length; ++item) augmented[column][item] /= divisor;
    for (let row = 0; row < vector.length; ++row) {
      if (row === column) continue;
      const multiplier = augmented[row][column];
      for (let item = column; item <= vector.length; ++item) augmented[row][item] -= multiplier * augmented[column][item];
    }
  }
  return augmented.map(row => row.at(-1));
}

export function fitLossModel(points) {
  const accepted = points.filter(point => point.steady);
  if (accepted.length < 4 || ![-1, 1].every(sign => accepted.filter(point => Math.sign(point.velocity_rad_s) === sign).length >= 2)) {
    return { valid: false, reason: '至少需要正反方向各两个合格稳态点', accepted_points: accepted.length };
  }
  const normal = Array.from({ length: 3 }, () => [0, 0, 0]);
  const rhs = [0, 0, 0];
  for (const point of accepted) {
    const row = [Math.sign(point.velocity_rad_s), point.velocity_rad_s, 1];
    for (let i = 0; i < 3; ++i) {
      rhs[i] += row[i] * point.command_ma;
      for (let j = 0; j < 3; ++j) normal[i][j] += row[i] * row[j];
    }
  }
  let coefficients;
  try { coefficients = solve(normal, rhs); }
  catch (error) { return { valid: false, reason: error.message, accepted_points: accepted.length }; }
  const [coulomb, viscous, bias] = coefficients;
  const residuals = accepted.map(point => {
    const predicted = Math.sign(point.velocity_rad_s) * coulomb + viscous * point.velocity_rad_s + bias;
    return { source: point.source, predicted_command_ma: predicted, residual_ma: point.command_ma - predicted };
  });
  const warnings = [];
  if (coulomb < 0 || viscous < 0) warnings.push('出现负损耗系数，线性模型或稳态数据可能不适用，请勿直接作为补偿参数');
  return {
    valid: true, model: 'command_ma = sign(velocity_rad_s) * coulomb_ma + viscous_ma_per_rad_s * velocity_rad_s + bias_ma',
    accepted_points: accepted.length, coulomb_ma: coulomb,
    viscous_ma_per_rad_s: viscous, viscous_ua_per_rad_s: viscous * 1000,
    bias_ma: bias, forward_coulomb_ma: coulomb + bias, reverse_coulomb_ma: coulomb - bias,
    direction_coulomb_difference_ma: 2 * bias,
    residual_rms_ma: Math.sqrt(mean(residuals.map(row => row.residual_ma ** 2))),
    residual_max_abs_ma: Math.max(...residuals.map(row => Math.abs(row.residual_ma))),
    residuals, warnings,
  };
}

function directionDifferences(points) {
  const magnitudes = [...new Set(points.map(point => Math.abs(point.target_velocity_rad_s)))].sort((a, b) => a - b);
  return magnitudes.flatMap(speed => {
    const positive = points.filter(point => point.steady && point.target_velocity_rad_s === speed);
    const negative = points.filter(point => point.steady && point.target_velocity_rad_s === -speed);
    if (!positive.length || !negative.length) return [];
    const forward = mean(positive.map(point => point.command_ma));
    const reverse = -mean(negative.map(point => point.command_ma));
    return [{ target_speed_rad_s: speed, forward_command_ma: forward, reverse_magnitude_ma: reverse,
      forward_minus_reverse_ma: forward - reverse,
      forward_repeats: positive.length, reverse_repeats: negative.length,
      forward_repeat_std_ma: deviation(positive.map(point => point.command_ma)),
      reverse_repeat_std_ma: deviation(negative.map(point => point.command_ma)),
    }];
  });
}

function parseArgs(argv) {
  const args = {};
  for (let index = 0; index < argv.length; ++index) {
    const name = argv[index];
    if (['--analyze-only', '--help'].includes(name)) { args[name.slice(2)] = true; continue; }
    if (!['--serial', '--elf', '--out-dir', '--speeds', '--repeats'].includes(name) || !argv[index + 1] || argv[index + 1].startsWith('--')) {
      throw new Error(`未知参数或缺少值：${name}`);
    }
    args[name.slice(2)] = argv[++index];
  }
  if (args.help) return args;
  if (!/^[0-9a-fA-F]{24}$/.test(args.serial ?? '') || !args.elf || !args['out-dir']) {
    throw new Error('必需参数：--serial <24 位 ST-Link 序列号> --elf <已烧录的诊断 ELF> --out-dir <数据目录>');
  }
  args.speeds = (args.speeds ?? '3,6,12,20,30').split(',').map(Number);
  args.repeats = Number(args.repeats ?? 2);
  if (!args.speeds.length || !args.speeds.every(value => Number.isFinite(value) && value > 0 && value <= 60) ||
      !Number.isInteger(args.repeats) || args.repeats < 1 || args.repeats > 20) {
    throw new Error('--speeds 必须为逗号分隔的正速度（最高 60 rad/s）；--repeats 必须为 1～20 的整数');
  }
  args.speeds = [...new Set(args.speeds)].sort((a, b) => a - b);
  return args;
}

function collectOne(argumentsList) {
  return new Promise((resolveRun, reject) => {
    const child = spawn(process.execPath, [fileURLToPath(new URL('./motor_diagnostics.mjs', import.meta.url)), ...argumentsList], {
      windowsHide: true, stdio: 'inherit',
    });
    child.once('error', reject);
    child.once('exit', (code, signal) => code === 0 ? resolveRun() : reject(new Error(`诊断失败：退出码 ${code}，信号 ${signal ?? '无'}`)));
  });
}

function summaryText(report) {
  const format = value => Number.isFinite(value) ? value.toFixed(3) : '—';
  const lines = [
    `无摩擦损耗辨识：${report.points.length} 次采集，${report.points.filter(point => point.steady).length} 次稳态合格。`,
    '窗口：3350 ≤ t < 5350 ms；优先采用 1 kHz 累积均值，按实际采样间隔加权；目标为速度 PI 的命令电流。',
    '速度目标/实测(rad/s) | 电流均值±标准差(mA) | 加速度(rad/s²) | 结果',
    ...report.points.map(point => `${format(point.target_velocity_rad_s)}/${format(point.velocity_rad_s)} | ${format(point.command_ma)} ± ${format(point.command_std_ma)} | ${format(point.acceleration_rad_s2)} | ${point.steady ? '稳态' : point.reasons.join('；')}`),
  ];
  if (report.fit.valid) {
    lines.push(`拟合：库仑摩擦 ${format(report.fit.coulomb_ma)} mA；黏性损耗 ${format(report.fit.viscous_ua_per_rad_s)} µA/(rad/s)；方向偏置 ${format(report.fit.bias_ma)} mA。`);
    lines.push(`拟合残差 RMS ${format(report.fit.residual_rms_ma)} mA，最大绝对残差 ${format(report.fit.residual_max_abs_ma)} mA；正反库仑摩擦差 ${format(report.fit.direction_coulomb_difference_ma)} mA。`);
    lines.push(...report.fit.warnings);
  } else lines.push(`未生成有效拟合：${report.fit.reason}。`);
  lines.push('本脚本仅测量和拟合，没有保存参数或烧录；拟合未包含齿槽转矩及温度变化。');
  return lines.join('\n') + '\n';
}

export async function main(argv = process.argv.slice(2)) {
  const args = parseArgs(argv);
  if (args.help) {
    console.log('node tools/autotune_frictionless.mjs --serial <24hex> --elf <diagnostic.elf> --out-dir <directory> [--speeds 3,6,12,20,30] [--repeats 2] [--analyze-only]');
    return;
  }
  const outDir = resolve(args['out-dir']);
  mkdirSync(outDir, { recursive: true });
  let interrupted = false;
  const onInterrupt = () => { interrupted = true; console.error('停止排队；等待当前有界诊断完成及关闭输出后生成已采集数据的报告。'); };
  process.on('SIGINT', onInterrupt);
  let collectionError;
  try {
    if (!args['analyze-only']) {
      if (!existsSync(args.elf)) throw new Error(`诊断 ELF 不存在：${args.elf}`);
      const plan = [];
      for (let repeat = 1; repeat <= args.repeats; ++repeat) {
        for (const magnitude of args.speeds) {
          // Reverse direction order on the second pass to expose heating/order bias.
          for (const sign of repeat % 2 ? [1, -1] : [-1, 1]) {
            const speed = magnitude * sign;
            const name = `capture-r${String(repeat).padStart(2, '0')}-${sign > 0 ? 'p' : 'n'}${String(magnitude).replace('.', 'p')}.json`;
            plan.push({ speed, path: join(outDir, name) });
          }
        }
      }
      if (plan.some(point => existsSync(point.path))) throw new Error('数据目录已有同名采集；请选新目录，或使用 --analyze-only 重分析。');
      for (const [index, point] of plan.entries()) {
        if (interrupted) break;
        console.log(`[${index + 1}/${plan.length}] ST-Link ${args.serial}，速度 ${point.speed} rad/s`);
        try {
          await collectOne(['--serial', args.serial, '--elf', resolve(args.elf), '--out', point.path,
            '--mode', 'off', '--current-ma', '0', '--speed-rad-s', String(point.speed), '--pulse-ms', '5000', '--duration-ms', '5400']);
        } catch (error) { collectionError = error.message; break; }
      }
    }
    const files = readdirSync(outDir).filter(name => /^capture-.*\.json$/.test(name)).sort();
    if (!files.length) throw new Error(collectionError ?? '目录内没有 capture-*.json 采集文件');
    const points = files.map(name => {
      const capture = JSON.parse(readFileSync(join(outDir, name), 'utf8'));
      if (capture.options?.serial?.toUpperCase() !== args.serial.toUpperCase()) throw new Error(`采集序列号不匹配：${name}`);
      return analyzeCapture(capture, name);
    });
    const report = {
      created_at: new Date().toISOString(), serial: args.serial, diagnostic_elf: resolve(args.elf),
      analyze_only: Boolean(args['analyze-only']), interrupted, collection_error: collectionError ?? null,
      fit_target: 'Steady commanded torque current mean (mA); instantaneous measured Iq feedback is noisier',
      points, fit: fitLossModel(points), direction_comparison: directionDifferences(points),
    };
    writeFileSync(join(outDir, 'loss-model.json'), JSON.stringify(report, null, 2) + '\n');
    const summary = summaryText(report);
    writeFileSync(join(outDir, 'loss-model.txt'), summary);
    console.log(summary);
    console.log(`报告：${join(outDir, 'loss-model.json')}`);
    if (collectionError) console.error(collectionError);
    if (collectionError || !report.fit.valid) process.exitCode = 1;
    if (interrupted) process.exitCode = 130;
  } finally { process.removeListener('SIGINT', onInterrupt); }
}

if (process.argv[1] && import.meta.url === pathToFileURL(resolve(process.argv[1])).href) {
  main().catch(error => { console.error(error.message); process.exitCode = 1; });
}
