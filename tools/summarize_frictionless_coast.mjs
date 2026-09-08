// Offline evidence only: reads capture JSON and writes JSON/Markdown reports.
// This module never launches another process or accesses motor hardware.
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readdirSync, readFileSync, statSync, writeFileSync } from 'node:fs';
import { join, relative, resolve } from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { analyzeRelease } from './optimize_frictionless.mjs';

const SETTLING_MS = 350;
const REFERENCE_MS = 200;
const STOP_SPEED_RAD_S = 0.2;
const STOP_HOLD_MS = 500;
const STOP_ANGLE_SPAN_RAD = 0.01;
const EXPECTED_SERIAL = '01290141524300143638414B';
const mean = values => values.length ? values.reduce((a, b) => a + b, 0) / values.length : null;
const finite = values => values.filter(Number.isFinite);
const peak = values => values.length ? Math.max(...values.map(Math.abs)) : null;
const readJson = path => JSON.parse(readFileSync(path, 'utf8'));
const hash = path => createHash('sha256').update(readFileSync(path)).digest('hex');
const meanVelocity = row => Number.isFinite(row.mean_velocity_rad_s) ? row.mean_velocity_rad_s : row.velocity;
const posix = path => path.replaceAll('\\', '/');

function periodMs(rows) {
  const intervals = rows.slice(1).map((row, i) => row.ms - rows[i].ms).filter(n => n > 0).sort((a, b) => a - b);
  return intervals.length ? intervals[Math.floor(intervals.length / 2)] : null;
}

/** A transient zero crossing is not a stop. Require both instantaneous and
 * interval-mean velocity to remain near zero for >=500 ms, and <0.01 rad of
 * angular movement over that confirming window. Onset is the first sampled
 * time meeting these conditions, not an interpolated physical stopping time.
 */
export function sustainedStop(rows, releaseMs) {
  const coast = rows.filter(row => row.ms >= releaseMs);
  const period = periodMs(rows);
  for (let begin = 0; begin < coast.length; ++begin) {
    let angleMin = Infinity, angleMax = -Infinity;
    for (let end = begin; end < coast.length; ++end) {
      const row = coast[end];
      if (!Number.isFinite(row.velocity) || !Number.isFinite(meanVelocity(row))
          || Math.abs(row.velocity) > STOP_SPEED_RAD_S || Math.abs(meanVelocity(row)) > STOP_SPEED_RAD_S
          || (end > begin && row.ms - coast[end - 1].ms > period * 1.5)) break;
      angleMin = Math.min(angleMin, row.angle);
      angleMax = Math.max(angleMax, row.angle);
      if (!Number.isFinite(angleMin) || !Number.isFinite(angleMax) || angleMax - angleMin > STOP_ANGLE_SPAN_RAD) break;
      if (row.ms - coast[begin].ms >= STOP_HOLD_MS) {
        const tail = coast.slice(begin);
        return {
          detected: true,
          onset_ms_after_release: coast[begin].ms - releaseMs,
          confirmed_ms_after_release: row.ms - releaseMs,
          confirming_sample_count: end - begin + 1,
          confirming_angle_span_rad: angleMax - angleMin,
          near_zero_through_capture_end: tail.every(sample => Math.abs(sample.velocity) <= STOP_SPEED_RAD_S
            && Math.abs(meanVelocity(sample)) <= STOP_SPEED_RAD_S),
        };
      }
    }
  }
  return { detected: false, onset_ms_after_release: null, confirmed_ms_after_release: null };
}

function sustainedReverse(rows, releaseMs, direction) {
  const coast = rows.filter(row => row.ms >= releaseMs);
  let first;
  for (const row of coast) {
    if (direction * meanVelocity(row) < -STOP_SPEED_RAD_S) {
      first ??= row.ms;
      if (row.ms - first >= STOP_HOLD_MS) return { detected: true, onset_ms_after_release: first - releaseMs };
    } else first = undefined;
  }
  return { detected: false, onset_ms_after_release: null };
}

function runtime(capture) {
  const m = capture.metrics ?? {};
  const cycles = m.runtime_control_max_cycles;
  const clock = m.SystemCoreClock;
  return {
    status: capture.status,
    deadline_miss_count_since_boot: m.app_control_deadline_miss_count ?? null,
    control_step_count_since_boot: m.app_control_step_count ?? null,
    cumulative_control_step_max_cycles: cycles ?? null,
    system_core_clock_hz: clock ?? null,
    cumulative_control_step_max_us: Number.isFinite(cycles) && clock > 0 ? cycles * 1e6 / clock : null,
    control_stack_min_words_since_boot: m.app_control_stack_min_words ?? null,
    maintenance_stack_min_words_since_boot: m.app_maintenance_stack_min_words ?? null,
    storage_stack_min_words_since_boot: m.app_storage_stack_min_words ?? null,
  };
}

export function summarizeCapture(capture) {
  const rows = capture.samples ?? [];
  const options = capture.options ?? {};
  const releaseMs = SETTLING_MS + options.pulse_ms;
  const reference = rows.filter(row => row.ms >= releaseMs - REFERENCE_MS && row.ms < releaseMs);
  const lastMs = rows.at(-1)?.ms;
  const final = rows.filter(row => row.ms >= lastMs - REFERENCE_MS);
  // Mean samples whose windows cross release still contain PI current/speed.
  const period = periodMs(rows);
  const coast = rows.filter(row => row.ms >= releaseMs);
  const completeCoastWindows = coast.filter(row => row.ms - period >= releaseMs);
  const direction = Math.sign(options.pulse_velocity_rad_s);
  const analysis = analyzeRelease(capture);
  const stop = sustainedStop(rows, releaseMs);
  const reverse = sustainedReverse(rows, releaseMs, direction);
  const finalSpeed = mean(final.map(meanVelocity));
  const observedMs = lastMs - releaseMs;
  return {
    options,
    runtime: runtime(capture),
    measurement_valid: analysis.valid,
    invalid_reasons: analysis.reasons,
    sample_count: rows.length,
    sample_period_ms: period,
    release_ms: releaseMs,
    observed_coast_duration_ms: observedMs,
    pre_release_mean_velocity_rad_s: mean(reference.map(meanVelocity)),
    pre_release_reference_sample_count: reference.length,
    pre_release_reference_start_ms: reference[0]?.ms ?? null,
    pre_release_reference_end_ms: reference.at(-1)?.ms ?? null,
    final_mean_velocity_rad_s: finalSpeed,
    final_mean_sample_count: final.length,
    final_mean_start_ms: final[0]?.ms ?? null,
    final_mean_end_ms: lastMs,
    final_speed_retention_ratio: finalSpeed / mean(reference.map(meanVelocity)),
    final_direction: Math.abs(finalSpeed) <= STOP_SPEED_RAD_S ? 'near_zero'
      : Math.sign(finalSpeed) === direction ? 'same_as_launch' : 'opposite_to_launch',
    sustained_stop: stop,
    sustained_reversal: reverse,
    coast_duration_until_first_sustained_stop_ms: stop.onset_ms_after_release ?? observedMs,
    coast_duration_is_lower_bound: !stop.detected,
    sampled_peak_abs_command_ma: peak(finite(rows.map(row => row.command_ma))),
    coast_sampled_peak_abs_command_ma: peak(finite(coast.map(row => row.command_ma))),
    coast_peak_abs_window_mean_command_ma: peak(finite(completeCoastWindows.map(row => row.mean_command_ma))),
    coast_peak_abs_window_mean_filtered_command_ma: peak(finite(completeCoastWindows.map(row => row.mean_filtered_command_ma))),
    same_direction_speed_retention_qualified: analysis.qualified,
    retention_failures: analysis.qualification_reasons ?? [],
    minimum_speed_ratio_after_handover: analysis.minimum_speed_ratio ?? null,
    maximum_speed_ratio_after_handover: analysis.maximum_speed_ratio ?? null,
    direction_normalized_slope_rad_s2: analysis.slope_rad_s2 ?? null,
    original_optimizer_analysis: analysis,
  };
}

function summarizeIdle(capture) {
  const rows = capture.samples ?? [];
  const angles = rows.map(row => row.angle);
  return {
    options: capture.options,
    runtime: runtime(capture),
    sample_count: rows.length,
    sampled_duration_ms: rows.at(-1)?.ms - rows[0]?.ms,
    angle_net_change_rad: angles.at(-1) - angles[0],
    angle_span_rad: Math.max(...angles) - Math.min(...angles),
    sampled_peak_abs_velocity_rad_s: peak(finite(rows.map(row => row.velocity))),
    peak_abs_window_mean_velocity_rad_s: peak(finite(rows.map(meanVelocity))),
    sampled_peak_abs_command_ma: peak(finite(rows.map(row => row.command_ma))),
    measurement_valid: capture.status === 2 && capture.metrics?.app_control_deadline_miss_count === 0
      && rows.length >= 8 && rows.at(-1)?.ms >= capture.options.duration_ms - Math.max(150, 3 * periodMs(rows)),
  };
}

function files(directory) {
  return existsSync(directory) ? readdirSync(directory).flatMap(name => {
    const path = join(directory, name);
    return statSync(path).isDirectory() ? files(path) : [path];
  }) : [];
}

function evidence(path, root) {
  return { file: posix(relative(root, path)), absolute_file: posix(resolve(path)), sha256: hash(path) };
}

function assertFinalParameters(capture, path) {
  const expected = { serial: EXPECTED_SERIAL, friction_ma: 22, viscous_microamps: 304,
    compensation_scale: 0.987, cogging_scale: 1, inertia_gain: 8, inertia_tau_ms: 10, current_ma: 0, dense: 0 };
  for (const [key, value] of Object.entries(expected)) {
    const actual = capture.options?.[key];
    if (typeof value === 'number' ? !Number.isFinite(actual) || Math.abs(actual - value) > 1e-6 : actual !== value) {
      throw new Error(`Unexpected ${key} in final evidence ${path}: ${actual}`);
    }
  }
}

const f = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '—';
const speedLabel = value => value > 0 ? `+${value}` : String(value);
const runtimeLine = entry => `${entry.runtime.status} / ${entry.runtime.deadline_miss_count_since_boot} / ${f(entry.runtime.cumulative_control_step_max_us, 2)}`;

function renderMarkdown(report) {
  const assisted = report.assisted;
  const lines = [
    '# 无摩擦模式最终滑行证据', '',
    `生成时间：${report.generated_at}。${report.complete ? '13 份预期记录均已读取。' : `报告尚未齐全：${report.pending.join('；')}。`}`,
    '',
    `此组参数延长了滑行，但没有通过全域保持释放前速度 50%–150% 的标准。当前 ${report.summary.assisted_qualified_count}/${assisted.length} 组通过该宽松筛选；它也不等价于理想无摩擦或没有自加速。`,
    '',
    '参数：Fc 22 mA、B 304 µA/(rad/s)、scale 0.987（有效 Fc 21.714 mA、B 300.048 µA/(rad/s)），惯量补偿 8、滤波 10 ms。低速肩部与角度补偿由被测 ELF 固定，报告不写入参数。',
    '',
    '| 启动目标 (rad/s) | 释放前均值 | 末段均值 | 末速比例 | 观测滑行 (s) | 持续停转/方向 | 50%–150% | 滑行采样电流峰值 (mA) |',
    '|---:|---:|---:|---:|---:|---|---|---:|',
  ];
  for (const entry of assisted) {
    const motion = entry.sustained_stop.detected ? `${f(entry.sustained_stop.onset_ms_after_release / 1000, 3)} s 起持续近零`
      : entry.final_direction === 'same_as_launch' ? '同向，未观察到持续停转' : entry.final_direction === 'opposite_to_launch' ? '末段反向' : '末段近零，未够持续判定';
    lines.push(`| ${speedLabel(entry.options.pulse_velocity_rad_s)} | ${f(entry.pre_release_mean_velocity_rad_s)} | ${f(entry.final_mean_velocity_rad_s)} | ${f(entry.final_speed_retention_ratio * 100, 1)}% | ${entry.coast_duration_is_lower_bound ? '≥' : ''}${f(entry.coast_duration_until_first_sustained_stop_ms / 1000, 3)} | ${motion} | ${entry.same_direction_speed_retention_qualified ? '通过' : '未通过'} | ${f(entry.coast_sampled_peak_abs_command_ma)} |`);
  }
  lines.push('', '低速逐项变化：', '');
  for (const entry of assisted.filter(entry => Math.abs(entry.options.pulse_velocity_rad_s) <= 6)) {
    lines.push(`- ${speedLabel(entry.options.pulse_velocity_rad_s)} rad/s：${f(entry.pre_release_mean_velocity_rad_s)} → ${f(entry.final_mean_velocity_rad_s)} rad/s；方向归一化斜率 ${f(entry.direction_normalized_slope_rad_s2, 3)} rad/s²；滑行速度范围为初速 ${f(entry.minimum_speed_ratio_after_handover * 100, 1)}%–${f(entry.maximum_speed_ratio_after_handover * 100, 1)}%。`);
  }
  lines.push('', '无助力对照为零 Iq 闭环，释放后命令电流为零，桥臂仍受控制。', '',
    '| 目标 (rad/s) | 释放前均值 | 末段均值 | 持续近零起点 (ms) | 确认时间 (ms) | 本模式观测时长/对照起点 |',
    '|---:|---:|---:|---:|---:|---:|');
  for (const entry of report.unassisted) {
    const comparison = report.comparisons.find(row => row.target_speed_rad_s === entry.options.pulse_velocity_rad_s);
    lines.push(`| ${speedLabel(entry.options.pulse_velocity_rad_s)} | ${f(entry.pre_release_mean_velocity_rad_s)} | ${f(entry.final_mean_velocity_rad_s, 4)} | ${f(entry.sustained_stop.onset_ms_after_release, 0)} | ${f(entry.sustained_stop.confirmed_ms_after_release, 0)} | ${comparison ? `${comparison.assisted_duration_is_lower_bound ? '≥' : ''}${f(comparison.observed_duration_ratio, 1)}×` : '—'} |`);
  }
  lines.push('', '时长比是这两次有限记录的描述性比较：起点由采样分辨率及阈值决定，不是通用性能保证。');
  lines.push('', '| 记录 | status / 累计超期数 / 累计最大控制步耗时 (µs) | 全程采样电流峰值 (mA) |', '|---|---:|---:|');
  for (const entry of [...assisted, ...report.unassisted]) lines.push(`| ${entry.kind} ${speedLabel(entry.options.pulse_velocity_rad_s)} | ${runtimeLine(entry)} | ${f(entry.sampled_peak_abs_command_ma)} |`);
  if (report.idle) {
    lines.push(`| 最终静置 | ${runtimeLine(report.idle)} | ${f(report.idle.sampled_peak_abs_command_ma)} |`, '',
      `最终静置：观测 ${f(report.idle.sampled_duration_ms / 1000, 3)} s，角度净变化 ${f(report.idle.angle_net_change_rad, 6)} rad，角度跨度 ${f(report.idle.angle_span_rad, 6)} rad，采样速度峰值 ${f(report.idle.sampled_peak_abs_velocity_rad_s, 4)} rad/s。`);
  } else lines.push('', '最终静置：等待 final-idle.json，当前没有最终静置结论。');
  lines.push('', '计算口径与限制：', '',
    '- 释放时刻 = 350 ms 静置 + pulse_ms。初速取释放前最后 200 ms 内、完全结束于释放前的区间均速；末速取最后一个样本前 200 ms 内的区间均速。长测仅有 1–2 个初速参考点，角度纹波仍影响均值。',
    '- 持续停转要求瞬时速度和区间均速均不超过 0.2 rad/s，连续覆盖至少 500 ms，且该确认窗内角度跨度不超过 0.01 rad。报告同时给出起点和确认时刻，未停转的滑行时长仅为观测下限。',
    '- 50%–150% 筛选沿用优化器：跳过释放后 200 ms，再检查全部区间均速，并禁止停转和反转。停转判定用于描述滑行时长，不能替代这一全曲线筛选。',
    '- 滑行与对照记录间隔约 32–125 ms，最终静置记录约 157 ms。command_ma 只反映采样点峰值，无法证明采样点之间的真实电流峰值；区间均值也不保留瞬时峰值。',
    '- 控制步最大周期数、超期数和栈余量均自启动累计，未在每次采集中清零；耗时按读取的 SystemCoreClock 换算。累计最大耗时不等于本次实验独有峰值，也不是整机 CPU 利用率。',
    '- 各目标/方向只有一次最终采集；初始角度、温度及速度纹波未独立随机化。不能由均值电流与均值速度还原精确机械功率或绝对摩擦。', '',
    '输入证据（相对本报告目录；完整路径、SHA-256、原始分析及所有数值见 JSON）：', '');
  for (const entry of [...assisted, ...report.unassisted, ...(report.idle ? [report.idle] : [])]) lines.push(`- [${entry.source.file}](<${entry.source.absolute_file}>)`);
  for (const config of report.configurations) lines.push(`- ${config.group} ELF SHA-256：\`${config.config.elf_sha256}\``);
  return lines.join('\n') + '\n';
}

export function generateReport(root) {
  const pending = [], assisted = [], unassisted = [], configurations = [];
  const specs = [
    { group: 'narrow-shoulder-long', speeds: [3, -3, 6, -6, 12, -12], duration_ms: 16000 },
    { group: 'narrow-shoulder-fast', speeds: [20, -20, 30, -30], duration_ms: 10000 },
  ];
  for (const spec of specs) {
    const summaryPath = join(root, spec.group, 'summary.json');
    const summary = existsSync(summaryPath) ? readJson(summaryPath) : null;
    if (summary?.config) configurations.push({ group: spec.group, config: summary.config });
    const directory = summary?.fingerprint ? join(root, spec.group, summary.fingerprint) : join(root, spec.group);
    const available = files(directory);
    for (const speed of spec.speeds) {
      const suffix = `${speed > 0 ? 'positive' : 'negative'}-${Math.abs(speed)}.json`;
      const matches = available.filter(path => path.endsWith(`\\${suffix}`) || path.endsWith(`/${suffix}`));
      if (matches.length !== 1) { pending.push(`${spec.group}/${suffix}: ${matches.length ? 'ambiguous input' : 'missing'}`); continue; }
      const path = matches[0], capture = readJson(path);
      assertFinalParameters(capture, path);
      if (capture.options.duration_ms !== spec.duration_ms || capture.options.pulse_velocity_rad_s !== speed
          || capture.options.mode !== 12) throw new Error(`Unexpected capture configuration: ${path}`);
      assisted.push({ kind: spec.group, source: evidence(path, root), ...summarizeCapture(capture) });
    }
  }
  for (const direction of ['positive', 'negative']) {
    const path = join(root, `unassisted-${direction}6.json`);
    if (!existsSync(path)) { pending.push(`unassisted-${direction}6.json: missing`); continue; }
    const capture = readJson(path);
    if (capture.options.serial !== EXPECTED_SERIAL || capture.options.mode !== 4294967294
        || capture.options.pulse_velocity_rad_s !== (direction === 'positive' ? 6 : -6)
        || capture.options.current_ma !== 0) throw new Error(`Unexpected baseline configuration: ${path}`);
    unassisted.push({ kind: 'unassisted_zero_iq', source: evidence(path, root), ...summarizeCapture(capture) });
  }
  const idlePath = join(root, 'final-idle.json');
  let idle = null;
  if (existsSync(idlePath)) {
    const capture = readJson(idlePath);
    assertFinalParameters(capture, idlePath);
    if (capture.options.mode !== 12 || capture.options.pulse_velocity_rad_s !== 0 || capture.options.pulse_ms !== 0) {
      throw new Error(`Unexpected idle configuration: ${idlePath}`);
    }
    idle = { kind: 'final_idle', source: evidence(idlePath, root), ...summarizeIdle(capture) };
  }
  if (!idle) pending.push('final-idle.json: missing');
  const comparisons = unassisted.flatMap(baseline => {
    const active = assisted.find(entry => entry.options.pulse_velocity_rad_s === baseline.options.pulse_velocity_rad_s);
    const onset = baseline.sustained_stop.onset_ms_after_release;
    if (!active || !onset) return [];
    return [{ target_speed_rad_s: baseline.options.pulse_velocity_rad_s,
      baseline_sustained_stop_onset_ms: onset,
      baseline_stop_confirmation_ms: baseline.sustained_stop.confirmed_ms_after_release,
      assisted_coast_duration_ms: active.coast_duration_until_first_sustained_stop_ms,
      assisted_duration_is_lower_bound: active.coast_duration_is_lower_bound,
      observed_duration_ratio: active.coast_duration_until_first_sustained_stop_ms / onset }];
  });
  const entries = [...assisted, ...unassisted, ...(idle ? [idle] : [])];
  return {
    generated_at: new Date().toISOString(),
    purpose: 'Read-only final evidence summary; no motor operation, firmware change or settings write',
    complete: pending.length === 0,
    pending,
    expected_capture_count: 13,
    definitions: {
      settling_ms: SETTLING_MS, reference_window_ms: REFERENCE_MS, final_window_ms: REFERENCE_MS,
      velocity_source: 'mean_velocity_rad_s (interval mean), fallback velocity',
      stop: { max_abs_instant_and_mean_velocity_rad_s: STOP_SPEED_RAD_S, minimum_hold_ms: STOP_HOLD_MS,
        maximum_confirming_angle_span_rad: STOP_ANGLE_SPAN_RAD,
        onset_is: 'First sampled near-zero time in a confirmed continuous run; not interpolated physical onset' },
      qualification: { min_ratio: 0.5, max_ratio: 1.5, skip_after_release_ms: 200,
        stop_or_reverse_allowed: false, zero_friction_proven: false },
      peak_current: 'Sampled command_ma maximum absolute value, not a true between-samples maximum',
      runtime_counters: 'Cumulative since boot, not reset at each capture; max cycles / SystemCoreClock gives control-step duration, not CPU utilization',
      baseline: 'Zero-Iq closed-loop torque command after PI launch; bridge is not switched off',
    },
    configurations,
    summary: {
      assisted_qualified_count: assisted.filter(entry => entry.same_direction_speed_retention_qualified).length,
      all_assisted_qualified: assisted.length === 10 && assisted.every(entry => entry.same_direction_speed_retention_qualified),
      ideal_frictionless_proven: false,
      all_captures_valid: pending.length === 0 && entries.every(entry => entry.measurement_valid),
      all_status_2: entries.every(entry => entry.runtime.status === 2),
      maximum_cumulative_deadline_miss_count: Math.max(...entries.map(entry => entry.runtime.deadline_miss_count_since_boot)),
      maximum_read_cumulative_control_step_us: Math.max(...entries.map(entry => entry.runtime.cumulative_control_step_max_us)),
      limitations: ['Final low-speed and high-speed captures have one repetition per condition.',
        'Speed retention is not uniform across initial speed and direction.',
        'Normal telemetry does not measure mean(command * velocity), so it cannot establish exact mechanical power.',
        'Sampled maxima can miss transients.'],
    },
    assisted, unassisted, comparisons, idle,
  };
}

function selfTest() {
  const samples = values => values.map((v, i) => ({ ms: i * 100, velocity: v, mean_velocity_rad_s: v, angle: 0 }));
  assert.equal(sustainedStop(samples([2, 0, 2, 2, 2, 2, 2, 2]), 0).detected, false, 'single crossing is not a stop');
  assert.equal(sustainedStop(samples([2, 0, 0, 0, 0, 0]), 0).detected, false, '400 ms is not confirmed');
  const stop = sustainedStop(samples([2, 0, 0, 0, 0, 0, 0, 0]), 0);
  assert.equal(stop.onset_ms_after_release, 100);
  assert.equal(stop.confirmed_ms_after_release, 600);
  assert.equal(stop.near_zero_through_capture_end, true);
  const drift = samples([0, 0, 0, 0, 0, 0, 0]).map((r, i) => ({ ...r, angle: i * 0.02 }));
  assert.equal(sustainedStop(drift, 0).detected, false, 'moving angle is not stationary');
  const spikes = samples([0, 0, 0, 0, 0, 0, 0]).map(r => ({ ...r, velocity: 0.5 }));
  assert.equal(sustainedStop(spikes, 0).detected, false, 'averaging must not hide instantaneous motion');
  assert.equal(sustainedReverse(samples([1, -1, 1, 1, 1, 1, 1]), 0, 1).detected, false);
  assert.equal(sustainedReverse(samples([1, -1, -1, -1, -1, -1, -1]), 0, 1).detected, true);
  console.log('9 offline stop/reversal checks passed; no files or hardware accessed.');
}

export function main(argv = process.argv.slice(2)) {
  let root = fileURLToPath(new URL('../build/frictionless-sep08', import.meta.url));
  let requireComplete = false;
  for (let i = 0; i < argv.length; ++i) {
    if (argv[i] === '--self-test') { selfTest(); return; }
    if (argv[i] === '--require-complete') { requireComplete = true; continue; }
    if (argv[i] === '--data-dir' && argv[i + 1]) { root = resolve(argv[++i]); continue; }
    if (argv[i] === '--help') { console.log('node tools/summarize_frictionless_coast.mjs [--data-dir <capture root>] [--require-complete] [--self-test]\nReads final coast/baseline/idle JSON only; writes final-coast-report.json and .md. Never accesses hardware.'); return; }
    throw new Error(`Unknown or incomplete argument: ${argv[i]}`);
  }
  const report = generateReport(root);
  mkdirSync(root, { recursive: true });
  writeFileSync(join(root, 'final-coast-report.json'), JSON.stringify(report, null, 2) + '\n');
  writeFileSync(join(root, 'final-coast-report.md'), renderMarkdown(report));
  console.log(JSON.stringify({ complete: report.complete, pending: report.pending, summary: report.summary,
    comparisons: report.comparisons, report: posix(join(root, 'final-coast-report.md')) }, null, 2));
  if (requireComplete && !report.complete) process.exitCode = 1;
}

if (process.argv[1] && import.meta.url === pathToFileURL(resolve(process.argv[1])).href) {
  try { main(); } catch (error) { console.error(error.stack ?? error.message); process.exitCode = 1; }
}
