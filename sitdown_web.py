# -*- coding: utf-8 -*-
"""
趴下窗口左右柱坐标 + 机器人定位 网页可视化工具
用法: python sitdown_web.py <charge_manager日志> <reflective_column_node日志> [输出html]
不指定输出时, 默认写在当前目录 sitdown_windows.html
同时会调用 sitdown_windows.py 在同目录生成同名 .txt 文本报告
生成的 HTML 自包含(无外部依赖, 离线可直接打开):
  左侧窗口列表: 失败(无电流)置顶标红, 支持按结果筛选
  右侧: 选中窗口的 y/x 轨迹图(左柱/右柱/机器人定位) + 逐秒明细表
"""
import sys, re, math, bisect, io, os, json, subprocess

def parse_logs(cp, lp):
    """解析两份日志, 返回内嵌 HTML 的 data 字典"""
    # 窗口: 精调点(FINE_TUNE) -> sit_down_high -> 趴桩测试(趴下完成)
    # wins 元素: [t_fine, t_sdh, t_end, res]
    wins = []
    t_fine = t_sdh = None
    for l in open(cp, 'r', encoding='utf-8', errors='replace'):
        t = l[1:20]
        if '已到达精调点' in l or '到达精调点, Z 慢降' in l:
            if t_sdh is None:
                t_fine = t_fine or t  # 取最先出现的精调点标记
        elif '精调已到 sit_down_high' in l:
            t_sdh = t
            if t_fine is None:
                t_fine = t  # 无精调点标记时回退
        elif '次趴桩测试' in l and (t_sdh or t_fine):
            # sit_down_high 缺失时(旧版本程序无此日志)退化为用精调点时刻
            wins.append([t_fine, t_sdh or t_fine, t])
            t_fine = t_sdh = None

    events = []
    dets = []   # 电流检测事件: [t, 次数, 目标x, 目标y] (目标位置仅新版日志有)
    last_det = None
    for l in open(cp, 'r', encoding='utf-8', errors='replace'):
        t = l[1:20]
        if '获取电流成功' in l:
            events.append((t, 'success'))
        elif '检测到无充电电流' in l:
            events.append((t, 'noload'))
        elif '接收到的自主充电指令' in l or 'Receive ChargeEnable' in l:
            # 新版日志中文关键字; 旧版日志(如1677)只有英文 Receive ChargeEnable
            events.append((t, 'start'))
        if '检测电流 次数:' in l:
            m = re.search(r'检测电流 次数:(\d+)', l)
            last_det = [t, int(m.group(1)), None, None]
            dets.append(last_det)
        elif '检测电流时目标位置' in l and last_det is not None and last_det[0] == t:
            m = re.search(r'x:(-?[\d.]+), y:(-?[\d.]+)', l)
            if m:
                last_det[2], last_det[3] = float(m.group(1)), float(m.group(2))
    events.sort()
    eTs = [e[0] for e in events]

    def win_result(t_end, next_start):
        limit = next_start if next_start else '9999'
        i = bisect.bisect_right(eTs, t_end)
        while i < len(events) and events[i][0] < limit:
            if events[i][1] in ('success', 'noload'):
                return {'kind': events[i][1], 't': events[i][0][11:19]}
            i += 1
        return {'kind': 'unknown', 't': ''}

    for wi in range(len(wins)):
        nxt = wins[wi+1][0] if wi+1 < len(wins) else None
        wins[wi].append(win_result(wins[wi][2], nxt))

    ppat = re.compile(r'【step 2】所有簇中心坐标 \[(\d+)个\]: (.+)')
    cpat = re.compile(r'\[(-?\d+)\]\((-?[\d.]+),(-?[\d.]+)\)')
    pose_pat = re.compile(r'【step 3-3----】发布坐标 x: (-?[\d.]+), y: (-?[\d.]+)')
    # 旧版本日志(如1677)无 step 3-3, 机器人定位在 step 3-2 "激光雷达位置"行(未含发布偏移)
    pose2_pat = re.compile(r'激光雷达位置 x: (-?[\d.]+), y: (-?[\d.]+)')
    frames, poses, poses2 = [], [], []
    for l in open(lp, 'r', encoding='utf-8', errors='replace'):
        m = ppat.search(l)
        if m:
            cs = [(float(a), float(b)) for _, a, b in cpat.findall(m.group(2))]
            frames.append((l[1:20], int(m.group(1)), cs))
            continue
        pm = pose_pat.search(l)
        if pm:
            poses.append((l[1:20], float(pm.group(1)), float(pm.group(2))))
            continue
        pm2 = pose2_pat.search(l)
        if pm2:
            poses2.append((l[1:20], float(pm2.group(1)), float(pm2.group(2))))
    if not poses:
        poses = poses2  # 无 3-3 时退化为 step 3-2 机器人位姿
    poses2 = None
    frames.sort(key=lambda f: f[0])
    poses.sort(key=lambda p: p[0])
    ts = [f[0] for f in frames]
    poseTs = [p[0] for p in poses]

    def track(seg):
        out = []
        prevL = prevR = None
        for t, n, cs in seg:
            L = R = None
            negs = [c for c in cs if c[1] < 0]
            poss = [c for c in cs if c[1] > 0]
            if len(negs) == 1:
                L = negs[0]
            elif negs and prevL:
                L = min(negs, key=lambda c: math.hypot(c[0]-prevL[0], c[1]-prevL[1]))
            elif negs:
                L = negs[0]
            if len(poss) == 1:
                R = poss[0]
            elif poss and prevR:
                R = min(poss, key=lambda c: math.hypot(c[0]-prevR[0], c[1]-prevR[1]))
            elif poss:
                R = poss[0]
            if L: prevL = L
            if R: prevR = R
            out.append((t, L, R, n))
        return out

    def r2(x):  # round to cm 精度两位, 减小json体积
        return None if x is None else round(x * 100) / 100.0

    windows = []
    for wi, w in enumerate(wins):
        t0, tS, t1, res = w
        i0 = bisect.bisect_left(ts, t0); i1 = bisect.bisect_right(ts, t1)
        t2 = t1[:14] + f'{int(t1[14:16]):02d}:{min(int(t1[17:19])+15, 59):02d}'
        i2 = bisect.bisect_right(ts, t2)
        tr = track(frames[i0:i2])
        pose_map = {}
        for pt in poses[bisect.bisect_left(poseTs, t0):bisect.bisect_right(poseTs, t2)]:
            pose_map.setdefault(pt[0][:19], (pt[1], pt[2]))
        rows = []
        shown = set()
        t0s = None
        for t, L, R, n in tr:
            kk = t[:19]
            if kk in shown: continue
            shown.add(kk)
            P = pose_map.get(kk)
            # 秒偏移(相对窗口首帧)
            hh, mm, ss = int(kk[11:13]), int(kk[14:16]), int(kk[17:19])
            sec = hh*3600 + mm*60 + ss
            if t0s is None: t0s = sec
            rows.append({
                't': kk[11:], 'dt': sec - t0s, 'n': n,
                'Lx': r2(L[0]) if L else None, 'Ly': r2(L[1]) if L else None,
                'Rx': r2(R[0]) if R else None, 'Ry': r2(R[1]) if R else None,
                'Px': r2(P[0]) if P else None, 'Py': r2(P[1]) if P else None,
            })
        # 汇总: 趴下动作过程内(sit_down_high -> 趴桩测试, 不含+15s)首末变化
        act = [r for r in rows if tS[11:19] <= r['t'] <= t1[11:19]]
        def delta(key):
            if len(act) < 2 or act[0][key] is None or act[-1][key] is None: return None
            return round((act[-1][key] - act[0][key]) * 1000)
        # 本窗口内的电流检测事件(精调点 ~ 趴下完成后+15s)
        wdets = [{'t': d[0][11:19], 'n': d[1],
                  'x': r2(d[2]) if d[2] is not None else None,
                  'y': r2(d[3]) if d[3] is not None else None}
                 for d in dets if t0 <= d[0] <= t2]
        windows.append({
            'idx': wi + 1, 't0': t0[11:19], 'tS': tS[11:19], 't1': t1[11:19],
            'kind': res['kind'], 'resT': res['t'],
            'dLy': delta('Ly'), 'dRy': delta('Ry'),
            'dets': wdets, 'rows': rows,
        })

    n_ok = sum(1 for w in windows if w['kind'] == 'success')
    n_nc = sum(1 for w in windows if w['kind'] == 'noload')
    n_unk = len(windows) - n_ok - n_nc
    # v5.0.22 起日志名带版本号(如 charge_manager_v5.0.22.2026_0907.log), 提取出来展示; 旧名无版本
    vm = re.search(r'_v(\d+\.\d+\.\d+)', os.path.basename(cp)) or \
        re.search(r'_v(\d+\.\d+\.\d+)', os.path.basename(lp))
    data = {'nWin': len(windows), 'nOk': n_ok, 'nNc': n_nc, 'nUnk': n_unk,
            'charge': os.path.basename(cp), 'loc': os.path.basename(lp),
            'ver': vm.group(1) if vm else None,
            'windows': windows}
    return data

def build_html(data):
    return HTML_TMPL.replace('__DATA__', json.dumps(data, ensure_ascii=False))

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    cp, lp = sys.argv[1], sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else 'sitdown_windows.html'
    data = parse_logs(cp, lp)
    with io.open(out, 'w', encoding='utf-8') as f:
        f.write(build_html(data))
    # 同步生成同名 txt 文本报告
    txt = os.path.splitext(out)[0] + '.txt'
    try:
        subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sitdown_windows.py'), cp, lp, txt], check=True)
    except Exception as e:
        sys.stderr.write(f'txt 报告生成失败: {e}\n')
    sys.stderr.write(f'完成: {os.path.abspath(out)}  (窗口 {data["nWin"]}, 成功 {data["nOk"]}, 无电流 {data["nNc"]})\n')

HTML_TMPL = r'''<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<title>趴下窗口分析</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { font: 13px/1.45 "Microsoft YaHei", sans-serif; color: #222; background: #f4f6f8; display: flex; height: 100vh; }
#side { width: 300px; background: #fff; border-right: 1px solid #dde; display: flex; flex-direction: column; }
#side header { padding: 10px 12px; border-bottom: 1px solid #dde; }
#side header h1 { font-size: 15px; margin-bottom: 4px; }
#stats { color: #667; font-size: 12px; }
#filters { padding: 8px 12px; border-bottom: 1px solid #dde; }
#filters button { padding: 3px 10px; margin-right: 6px; border: 1px solid #ccd; background: #fff; border-radius: 3px; cursor: pointer; font-size: 12px; }
#filters button.on { background: #2a6; color: #fff; border-color: #2a6; }
#list { overflow-y: auto; flex: 1; }
.witem { padding: 6px 12px; border-bottom: 1px solid #eef; cursor: pointer; display: flex; align-items: baseline; gap: 8px; }
.witem:hover { background: #f0f7ff; }
.witem.sel { background: #dcefff; }
.witem .no { color: #889; width: 36px; }
.witem .tm { font-family: "Segoe UI", "Microsoft YaHei", sans-serif; font-variant-numeric: tabular-nums; }
.witem .badge { margin-left: auto; font-size: 11px; padding: 1px 7px; border-radius: 8px; color: #fff; }
.b-noload { background: #d43f3a; } .b-success { background: #3c9a3c; } .b-unknown { background: #999; }
#main { flex: 1; overflow-y: auto; padding: 14px 18px; }
#winTitle { font-size: 15px; margin-bottom: 10px; }
#winTitle .badge { font-size: 12px; padding: 2px 9px; border-radius: 9px; color: #fff; margin-left: 8px; }
h3 { margin: 14px 0 6px; font-size: 13px; color: #334; }
.plot { background: #fff; border: 1px solid #dde; border-radius: 4px; padding: 8px; position: relative; }
.ctip { position: absolute; display: none; background: #fffde8; border: 1px solid #ccb; border-radius: 4px; padding: 5px 9px; font: 12.5px/1.55 "Segoe UI", "Microsoft YaHei", sans-serif; font-variant-numeric: tabular-nums; pointer-events: none; white-space: pre; box-shadow: 2px 2px 6px rgba(0,0,0,.15); z-index: 5; top: 12px; }
.legend { font-size: 12px; margin: 4px 2px 0; }
.legend span { margin-right: 14px; white-space: nowrap; }
.sw { display: inline-block; width: 18px; height: 3px; vertical-align: middle; margin-right: 4px; }
table { border-collapse: collapse; background: #fff; font-family: "Segoe UI", "Microsoft YaHei", sans-serif; font-variant-numeric: tabular-nums; font-size: 12.5px; margin-top: 4px; }
th, td { border: 1px solid #dde; padding: 3px 9px; text-align: right; }
th { background: #eef2f7; font-family: "Microsoft YaHei"; }
td.t, th.t { text-align: left; }
.miss { color: #d43f3a; font-weight: bold; }
</style>
</head>
<body>
<div id="side">
  <header>
    <h1>趴下窗口分析</h1>
    <div id="stats"></div>
  </header>
  <div id="filters">
    <button data-f="noload" class="on">失败(无电流)</button>
    <button data-f="success">成功</button>
    <button data-f="all">全部</button>
  </div>
  <div id="list"></div>
</div>
<div id="main">
  <div id="winTitle"></div>
  <h3>Y 轨迹 (mm)　<span style="color:#889;font-weight:normal">正 y = 机器人左移方向</span></h3>
  <div class="plot"><div id="marksY" style="position:relative;height:18px;font-size:11px;color:#445;white-space:nowrap;overflow:visible"></div><svg id="plotY" width="100%" height="260"></svg><div class="legend" id="legY"></div></div>
  <h3>X 轨迹 (mm)</h3>
  <div class="plot"><div id="marksX" style="position:relative;height:18px;font-size:11px;color:#445;white-space:nowrap;overflow:visible"></div><svg id="plotX" width="100%" height="200"></svg><div class="legend" id="legX"></div></div>
  <h3>逐秒明细（精调点 → 趴下完成 → 之后 15s）</h3>
  <div id="tbl"></div>
</div>
<script>
const D = __DATA__;
const KIND = { noload: ['失败·无电流', '#d43f3a'], success: ['成功', '#3c9a3c'], unknown: ['无结论', '#999'] };
const CL = { L: '#1a6fcc', R: '#e08a00', P: '#9b30d9' };  // 左柱/右柱/机器人定位
let curFilter = 'noload', selIdx = null;

const stats = document.getElementById('stats');
stats.textContent = `窗口 ${D.nWin}　成功 ${D.nOk}　无电流 ${D.nNc}　无结论 ${D.nUnk}`;
stats.appendChild(document.createElement('br'));
stats.append(D.charge + ' / ' + D.loc + (D.ver ? `  [v${D.ver}]` : ''));
if (location.protocol.startsWith('http')) {
  const a = document.createElement('a');
  a.href = '/txt'; a.textContent = '下载 txt 报告';
  a.style.cssText = 'color:#1a6fcc;margin-left:10px';
  stats.append(a);
}

function buildList() {
  const list = document.getElementById('list');
  list.innerHTML = '';
  // 失败置顶, 其余按时间(原序)
  let order = [];
  D.windows.forEach(w => { if (w.kind === 'noload') order.push(w); });
  D.windows.forEach(w => { if (w.kind !== 'noload') order.push(w); });
  order.forEach(w => {
    if (curFilter === 'noload' && w.kind !== 'noload') return;
    if (curFilter === 'success' && w.kind !== 'success') return;
    const el = document.createElement('div');
    el.className = 'witem' + (selIdx === w.idx ? ' sel' : '');
    const [lab, col] = KIND[w.kind];
    el.innerHTML = `<span class="no">#${w.idx}</span><span class="tm">${w.t0}~${w.t1}</span>` +
      `<span class="badge b-${w.kind}" title="${lab} ${w.resT}">${lab}</span>`;
    el.onclick = () => select(w.idx);
    list.appendChild(el);
  });
}
document.querySelectorAll('#filters button').forEach(b => {
  b.onclick = () => {
    curFilter = b.dataset.f;
    document.querySelectorAll('#filters button').forEach(x => x.classList.toggle('on', x === b));
    buildList();
  };
});

function select(idx) {
  selIdx = idx;
  buildList();
  const w = D.windows.find(x => x.idx === idx);
  const [lab, col] = KIND[w.kind];
  document.getElementById('winTitle').innerHTML =
    `窗口 #${idx}　${w.t0} 精调点 → ${w.tS} sit_down_high → ${w.t1} 趴下完成　` +
    `<span class="badge" style="background:${col}">${lab} ${w.resT}</span>` +
    `　<span style="color:#667">趴下过程 Δy: 左柱 ${fmt(w.dLy)} / 右柱 ${fmt(w.dRy)} mm</span>` +
    detLine(w);
  drawPlot('plotY', 'legY', w, 'y');
  drawPlot('plotX', 'legX', w, 'x');
  drawTable(w);
}
function fmt(v) { return v === null || v === undefined ? '--' : (v > 0 ? '+' : '') + v; }
// 电流检测事件行: 第i次 + 时刻 + 目标位置(新版日志有)
const DET_COL = '#00796b';
function detLine(w) {
  if (!w.dets || !w.dets.length) return '';
  const parts = w.dets.map(d =>
    `第${d.n}次 ${d.t}` + (d.x !== null ? ` (目标x ${(d.x*1000).toFixed(0)}, y ${(d.y*1000).toFixed(0)}mm)` : ''));
  return `<div style="font-size:12px;color:${DET_COL};margin-top:3px">◆ 电流检测 ${w.dets.length}次: ${parts.join(' · ')}</div>`;
}

// ---- 轻量折线图 ----
function drawPlot(svgId, legId, w, axis) {
  const svg = document.getElementById(svgId);
  const W = svg.clientWidth || 900, H = +svg.getAttribute('height');
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  svg.innerHTML = '';
  const ML = 56, MR = 16, MT = 12, MB = 26;
  const series = [
    { name: axis === 'y' ? '左柱 y' : '左柱 x', color: CL.L, key: axis === 'y' ? 'Ly' : 'Lx' },
    { name: axis === 'y' ? '右柱 y' : '右柱 x', color: CL.R, key: axis === 'y' ? 'Ry' : 'Rx' },
    { name: axis === 'y' ? '机器人定位 y' : '机器人定位 x', color: CL.P, key: axis === 'y' ? 'Py' : 'Px' },
  ];
  let t1s = w.t1;
  let vals = [];
  w.rows.forEach(r => { series.forEach(s => { if (r[s.key] !== null) vals.push(r[s.key] * 1000); }); });
  if (!vals.length) vals = [0];
  let vmin = Math.min(...vals), vmax = Math.max(...vals);
  const pad = Math.max((vmax - vmin) * 0.12, 10);
  vmin -= pad; vmax += pad;
  const dtMax = Math.max(...w.rows.map(r => r.dt), 1);
  // 轴
  const NS = 'http://www.w3.org/2000/svg';
  function el(tag, attrs, txt) {
    const e = document.createElementNS(NS, tag);
    for (const k in attrs) e.setAttribute(k, attrs[k]);
    if (txt !== undefined) e.textContent = txt;
    svg.appendChild(e); return e;
  }
  const xOf = dt => ML + (W - ML - MR) * dt / dtMax;
  const yOf = v => MT + (H - MT - MB) * (1 - (v - vmin) / (vmax - vmin));
  // 图上文字统一加白色描边底, 避免与曲线重叠时看不清
  const HALO = { 'paint-order': 'stroke', stroke: '#fff', 'stroke-width': 3, 'stroke-linejoin': 'round' };
  function elText(attrs, txt) {
    return el('text', Object.assign({}, HALO, attrs), txt);
  }
  // 网格 + y轴刻度
  for (let g = 0; g <= 4; g++) {
    const v = vmin + (vmax - vmin) * g / 4;
    el('line', { x1: ML, y1: yOf(v), x2: W - MR, y2: yOf(v), stroke: '#e6ebf0' });
    el('text', { x: ML - 6, y: yOf(v) + 4, 'text-anchor': 'end', 'font-size': 10, fill: '#788' }, Math.round(v));
  }
  // x轴刻度(每5s)
  for (let t = 0; t <= dtMax; t += 5) {
    el('line', { x1: xOf(t), y1: H - MB, x2: xOf(t), y2: H - MB + 4, stroke: '#aab' });
    el('text', { x: xOf(t), y: H - MB + 14, 'text-anchor': 'middle', 'font-size': 10, fill: '#788' }, t + 's');
  }
  // 阶段标注: 竖线在图内, 标签文字放图上方专用行(与竖线位置对齐)
  const marks = [
    { t: w.t0, lab: '精调点', col: '#3c9a3c' },
    { t: w.tS, lab: 'sit_down_high', col: '#1a6fcc' },
    { t: w.t1, lab: '趴下完成', col: '#d43f3a' },
  ];
  const marksBox = document.getElementById(legId.replace('leg', 'marks'));
  if (marksBox) {
    marksBox.innerHTML = '';
    const spans = [];
    marks.forEach(m => {
      const row = w.rows.find(r => r.t >= m.t);
      if (!row) return;
      const x = xOf(row.dt);
      const pct = (x / W * 100).toFixed(1);
      const sp = document.createElement('span');
      sp.style.cssText = `position:absolute;left:${pct}%;transform:translateX(-50%);white-space:nowrap;color:${m.col};font-weight:600`;
      sp.textContent = `▾ ${m.lab}`;
      marksBox.appendChild(sp);
      spans.push(sp);
    });
    // 水平重叠检测: 与同行已放标签重叠时下移一行(上下错开)
    let twoRows = false;
    // span 用 translateX(-50%), offsetLeft 即视觉中心; 视觉区间 = 中心 ± 半宽
    const rects = spans.map(sp => {
      const c = sp.offsetLeft;
      const hw = sp.offsetWidth / 2;
      return { l: c - hw, r: c + hw, row: 0 };
    });
    for (let i = 0; i < spans.length; i++) {
      for (let j = 0; j < i; j++) {
        if (rects[j].row === rects[i].row && rects[i].l < rects[j].r - 2 && rects[j].l < rects[i].r - 2) {
          rects[i].row = 1;
          spans[i].style.top = '10px';
          twoRows = true;
        }
      }
    }
    marksBox.style.height = twoRows ? '28px' : '18px';
  }
  marks.forEach(m => {
    const row = w.rows.find(r => r.t >= m.t);
    if (!row) return;
    const x = xOf(row.dt);
    el('line', { x1: x, y1: MT, x2: x, y2: H - MB, stroke: m.col, 'stroke-dasharray': m === marks[2] ? '5,4' : '3,3', 'stroke-width': 1.2 });
  });
  // 趴下完成时刻: 各序列值标注在曲线旁(带防碰撞)
  const endRow = w.rows.find(r => r.t >= w.t1);
  if (endRow) {
    const nearRight = xOf(endRow.dt) > W - MR - 60;
    const x = xOf(endRow.dt);
    // 收集有值序列, 按 y 从小到大排, 强制最小间距 13px
    const items = [];
    series.forEach(s => {
      const v = endRow[s.key];
      if (v === null || v === undefined) return;
      items.push({ y: yOf(v * 1000), txt: (v * 1000).toFixed(0), col: s.color });
    });
    items.sort((a, b) => a.y - b.y);
    for (let i = 1; i < items.length; i++) {
      if (items[i].y - items[i-1].y < 13) items[i].y = items[i-1].y + 13;
    }
    // 越界的整体回推
    const over = items.length ? items[items.length-1].y - (H - MB) : 0;
    if (over > 0) items.forEach(it => it.y -= over);
    items.forEach(it => {
      elText({ x: nearRight ? x - 8 : x + 8, y: Math.max(it.y - 5, MT + 10), 'font-size': 10.5, 'font-weight': 'bold',
               fill: it.col, 'text-anchor': nearRight ? 'end' : 'start' }, it.txt);
    });
  }
  // 电流检测事件: 在机器人定位曲线上标菱形+检N+坐标值; 定位缺失时标在图顶部
  (w.dets || []).forEach(d => {
    const row = w.rows.find(r => r.t >= d.t);
    if (!row) return;
    const x = xOf(row.dt);
    const v = row[axis === 'y' ? 'Py' : 'Px'];
    if (v !== null && v !== undefined) {
      const y = yOf(v * 1000);
      el('path', { d: `M${x},${y-5} L${x+5},${y} L${x},${y+5} L${x-5},${y} Z`, fill: DET_COL, stroke: '#fff', 'stroke-width': 1 });
      // 靠右边界时标签放左侧, 避免出图
      const nearRight = x > W - MR - 50;
      elText({ x: nearRight ? x - 8 : x + 8, y: y - 7, 'font-size': 10, 'font-weight': 'bold',
               fill: DET_COL, 'text-anchor': nearRight ? 'end' : 'start' },
             `检${d.n} ${(v * 1000).toFixed(0)}`);
    } else {
      el('path', { d: `M${x},${MT} L${x+4},${MT+7} L${x-4},${MT+7} Z`, fill: DET_COL });
      elText({ x: x, y: MT + 18, 'font-size': 10, 'font-weight': 'bold', fill: DET_COL, 'text-anchor': 'middle' }, `检${d.n}`);
    }
  });
  // 0线
  if (vmin < 0 && vmax > 0) el('line', { x1: ML, y1: yOf(0), x2: W - MR, y2: yOf(0), stroke: '#c33', 'stroke-width': 0.8, opacity: 0.5 });
  // 序列(纯展示, 交互由底部 overlay 统一处理)
  series.forEach(s => {
    let d = '', pen = false;
    w.rows.forEach(r => {
      const v = r[s.key];
      if (v === null) { pen = false; return; }
      const p = `${pen ? 'L' : 'M'}${xOf(r.dt).toFixed(1)},${yOf(v * 1000).toFixed(1)}`;
      d += (pen ? ' ' : '') + p; pen = true;
      el('circle', { cx: xOf(r.dt), cy: yOf(v * 1000), r: 2.4, fill: s.color, 'pointer-events': 'none' });
    });
    if (d) el('path', { d, fill: 'none', stroke: s.color, 'stroke-width': 1.8, 'pointer-events': 'none' });
  });

  // 竖线跟随交互: 鼠标左右移动定位到最近一秒, 显示该时刻各序列值
  svg.style.cursor = 'crosshair';
  const box = svg.parentElement;
  box.querySelectorAll('.ctip').forEach(x => x.remove());
  const tip = document.createElement('div');
  tip.className = 'ctip';
  box.appendChild(tip);
  const vline = el('line', { y1: MT, y2: H - MB, stroke: '#445', 'stroke-width': 1, 'stroke-dasharray': '4,3', visibility: 'hidden', 'pointer-events': 'none' });
  el('rect', { x: ML, y: MT, width: W - ML - MR, height: H - MT - MB, fill: 'transparent' });
  svg.addEventListener('mousemove', e => {
    const rect = svg.getBoundingClientRect();
    const sx = W / rect.width;
    const mx = (e.clientX - rect.left) * sx;
    if (mx < ML || mx > W - MR) { vline.setAttribute('visibility', 'hidden'); tip.style.display = 'none'; return; }
    const dt = (mx - ML) / (W - ML - MR) * dtMax;
    let best = null, bd = 1e9;
    w.rows.forEach(r => { const dd = Math.abs(r.dt - dt); if (dd < bd) { bd = dd; best = r; } });
    if (!best) return;
    const vx = xOf(best.dt);
    vline.setAttribute('x1', vx); vline.setAttribute('x2', vx);
    vline.setAttribute('visibility', 'visible');
    let txt = `${best.t}  (+${best.dt}s)  簇${best.n}`;
    series.forEach(s => {
      const v = best[s.key];
      txt += `\n${s.name}: ` + (v === null ? '--' : (v * 1000).toFixed(0) + 'mm');
    });
    tip.textContent = txt;
    tip.style.display = 'block';
    const prect = box.getBoundingClientRect();
    let left = vx / sx + rect.left - prect.left + 12;
    if (left + tip.offsetWidth > prect.width - 4) left = vx / sx + rect.left - prect.left - tip.offsetWidth - 12;
    tip.style.left = Math.max(2, left) + 'px';
  });
  svg.addEventListener('mouseleave', () => { vline.setAttribute('visibility', 'hidden'); tip.style.display = 'none'; });
  document.getElementById(legId).innerHTML =
    series.map(s => `<span><span class="sw" style="background:${s.color}"></span>${s.name}</span>`).join('');
}

function drawTable(w) {
  const rows = w.rows.map(r =>
    `<tr${r.Ry === null && r.Rx === null && r.n > 0 ? ' style="background:#fff3f3"' : ''}>` +
    `<td class="t">${r.t}</td><td>${r.n}</td>` +
    pair(r.Lx, r.Ly) + pair(r.Rx, r.Ry) + pair(r.Px, r.Py) + `</tr>`).join('');
  document.getElementById('tbl').innerHTML =
    `<table><tr><th class="t">时刻</th><th>簇数</th><th>左柱(x,y)mm</th><th>右柱(x,y)mm</th><th>机器人定位(x,y)mm</th></tr>${rows}</table>`;
}
function pair(x, y) {
  return (x === null || y === null) ? '<td class="miss">--</td>' : `<td>(${Math.round(x*1000)}, ${Math.round(y*1000)})</td>`;
}

// 默认选第一个失败窗口, 否则第一个窗口
const first = D.windows.find(w => w.kind === 'noload') || D.windows[0];
select(first ? first.idx : 0);
window.addEventListener('resize', () => { if (selIdx !== null) select(selIdx); });
</script>
</body>
</html>'''

if __name__ == '__main__':
    main()
