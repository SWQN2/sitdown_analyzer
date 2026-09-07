# -*- coding: utf-8 -*-
"""
趴下窗口左右柱坐标轨迹自动分析工具
用法: python sitdown_windows.py <charge_manager日志> <reflective_column_node日志> [输出文件]
不指定输出文件时, 默认写在当前目录 sitdown_windows.txt

输出内容:
  1. 概览: 窗口数 / 成功 / 无电流统计
  2. 汇总表: 每个趴下窗口首末帧左右柱坐标变化 + 电流结果
  3. 失败(无电流)窗口明细: 逐秒轨迹(趴下过程 + 15秒), 置于最前
  4. 全部窗口按时间顺序明细: 失败窗口同样包含并标注
"""
import sys, re, math, statistics, collections, bisect, io, os

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    cp, lp = sys.argv[1], sys.argv[2]
    out = sys.argv[3] if len(sys.argv) > 3 else 'sitdown_windows.txt'
    sys.stdout = io.open(out, 'w', encoding='utf-8')

    # 1. 趴下窗口: 精调点(FINE_TUNE) -> sit_down_high -> 趴桩测试(趴下完成)
    # wins 元素: [t_fine, t_sdh, t_end, res]
    wins = []
    t_fine = t_sdh = None
    for l in open(cp, 'r', encoding='utf-8', errors='replace'):
        t = l[1:20]
        if '已到达精调点' in l or '到达精调点, Z 慢降' in l:
            if t_sdh is None:
                t_fine = t_fine or t
        elif '精调已到 sit_down_high' in l:
            t_sdh = t
            if t_fine is None:
                t_fine = t
        elif '次趴桩测试' in l and (t_sdh or t_fine):
            # sit_down_high 缺失时(旧版本程序无此日志)退化为用精调点时刻
            wins.append([t_fine, t_sdh or t_fine, t])
            t_fine = t_sdh = None
    print(f'charge log : {cp}')
    print(f'loc log    : {lp}')
    # v5.0.22 起日志名带版本号(如 charge_manager_v5.0.22.2026_0907.log), 提取出来展示; 旧名无版本
    vm = re.search(r'_v(\d+\.\d+\.\d+)', os.path.basename(cp)) or \
        re.search(r'_v(\d+\.\d+\.\d+)', os.path.basename(lp))
    if vm:
        print(f'程序版本   : v{vm.group(1)} (取自日志文件名)')
    print(f'趴下窗口数: {len(wins)}')
    if not wins:
        print('未找到趴下窗口, 请确认日志内容')
        sys.exit(0)

    # 1b. 电流事件时间线: 每个窗口向后找第一个电流事件(截至下一窗口/新任务)
    events = []  # (t, kind)
    dets = []    # 电流检测事件: [t, 次数, 目标x, 目标y] (目标位置仅新版日志有)
    last_det = None
    for l in open(cp, 'r', encoding='utf-8', errors='replace'):
        t = l[1:20]
        if '获取电流成功' in l:
            events.append((t, '成功'))
        elif '检测到无充电电流' in l:
            events.append((t, '无电流'))
        elif '接收到的自主充电指令' in l or 'Receive ChargeEnable' in l:
            # 新版日志中文关键字; 旧版日志(如1677)只有英文 Receive ChargeEnable
            events.append((t, '任务开始'))
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
        """窗口结束(趴桩测试)后, 下一个趴下/新任务前的第一个电流事件"""
        limit = next_start if next_start else '9999'
        i = bisect.bisect_right(eTs, t_end)
        while i < len(events) and events[i][0] < limit:
            if events[i][1] in ('成功', '无电流'):
                return events[i][1] + f'@{events[i][0][11:19]}'
            i += 1
        return '无结论(超时/中断)'

    for wi in range(len(wins)):
        nxt = wins[wi+1][0] if wi+1 < len(wins) else None
        wins[wi].append(win_result(wins[wi][2], nxt))

    # 2. 每帧全簇中心 + 发布坐标(机器人定位值)
    ppat = re.compile(r'【step 2】所有簇中心坐标 \[(\d+)个\]: (.+)')
    cpat = re.compile(r'\[(-?\d+)\]\((-?[\d.]+),(-?[\d.]+)\)')
    pose_pat = re.compile(r'【step 3-3----】发布坐标 x: (-?[\d.]+), y: (-?[\d.]+)')
    # 旧版本日志(如1677)无 step 3-3, 机器人定位在 step 3-2 "激光雷达位置"行(未含发布偏移)
    pose2_pat = re.compile(r'激光雷达位置 x: (-?[\d.]+), y: (-?[\d.]+)')
    frames = []  # (t, n, [(x,y), ...])
    poses = []   # (t, x, y)
    poses2 = []
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
    print(f'全簇中心帧数: {len(frames)}, 发布坐标帧数: {len(poses)}')

    # 3. 帧级左右柱跟踪(带连续性选择)
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

    # 汇总表
    print('\n====== 所有窗口汇总: 趴下过程内左右柱坐标变化(首帧->末帧) ======')
    print(f'{"#":<4}{"窗口":<20}{"L首(x,y)":>18}{"L末(x,y)":>18}{"LΔx":>7}{"LΔy":>7} | {"R首(x,y)":>18}{"R末(x,y)":>18}{"RΔx":>7}{"RΔy":>7}{"簇数":>4}  结果')
    dLx=[];dLy=[];dRx=[];dRy=[];R_missing=[]
    for wi, w in enumerate(wins):
        t0, tS, t1 = w[0], w[1], w[2]
        # 汇总只看趴下动作段: sit_down_high -> 趴桩测试
        i0 = bisect.bisect_left(ts, tS); i1 = bisect.bisect_right(ts, t1)
        seg = frames[i0:i1]
        if len(seg) < 5:
            print(f'{wi+1:<4}{t0[11:]}~{t1[11:]}  (帧数不足: {len(seg)})')
            continue
        tr = track(seg)
        Ls = [x for x in tr if x[1]]; Rs = [x for x in tr if x[2]]
        Lf, Ll = Ls[0][1], Ls[-1][1] if Ls else (None, None)
        Rf, Rl = Rs[0][2], Rs[-1][2] if Rs else (None, None)
        def fmt(c): return f'({c[0]:+.3f},{c[1]:+.3f})' if c else '   --     '
        def d(a, b): return (b-a)*1000 if a is not None and b is not None else float('nan')
        dLx_ = d(Lf[0] if Lf else None, Ll[0] if Ll else None)
        dLy_ = d(Lf[1] if Lf else None, Ll[1] if Ll else None)
        dRx_ = d(Rf[0] if Rf else None, Rl[0] if Rf else None)
        dRy_ = d(Rf[1] if Rf else None, Rl[1] if Rf else None)
        nR = sum(1 for x in tr if x[2]); nfrm = len(tr)
        print(f'{wi+1:<4}{t0[11:]}~{t1[11:]}{fmt(Lf):>18}{fmt(Ll):>18}{dLx_:>+7.0f}{dLy_:>+7.0f} | {fmt(Rf):>18}{fmt(Rl):>18}{dRx_:>+7.0f}{dRy_:>+7.0f}{nR}/{nfrm:>4}  {wins[wi][3]}')
        if Lf and Ll: dLx.append(dLx_); dLy.append(dLy_)
        if Rf and Rl: dRx.append(dRx_); dRy.append(dRy_)
        if nR < nfrm * 0.5: R_missing.append(wi+1)

    if dLx:
        print(f'\n左柱(全窗口): Δx 均值{statistics.mean(dLx):+.1f}mm 中位{statistics.median(dLx):+.1f} 范围[{min(dLx):+.0f},{max(dLx):+.0f}]')
        print(f'            Δy 均值{statistics.mean(dLy):+.1f}mm 中位{statistics.median(dLy):+.1f} 范围[{min(dLy):+.0f},{max(dLy):+.0f}]')
    if dRx:
        print(f'右柱(有数据窗口): Δx 均值{statistics.mean(dRx):+.1f}mm 中位{statistics.median(dRx):+.1f} 范围[{min(dRx):+.0f},{max(dRx):+.0f}]')
        print(f'              Δy 均值{statistics.mean(dRy):+.1f}mm 中位{statistics.median(dRy):+.1f} 范围[{min(dRy):+.0f},{max(dRy):+.0f}]')
    if R_missing:
        print(f'右柱中途消失/不足半程的窗口: {R_missing}')

    n_ok = sum(1 for w in wins if w[3].startswith('成功'))
    n_nc = sum(1 for w in wins if w[3].startswith('无电流'))
    n_unk = len(wins) - n_ok - n_nc
    print(f'\n结果统计: 成功 {n_ok} / 无电流(失败) {n_nc} / 无结论 {n_unk}')

    # 明细轨迹
    def detail(wi, note):
        t0, tS, t1 = wins[wi][0], wins[wi][1], wins[wi][2]
        i0 = bisect.bisect_left(ts, t0); i1 = bisect.bisect_right(ts, t1)
        # 趴下完成后延长15秒看趴着阶段
        t2 = t1[:14] + f'{int(t1[14:16]):02d}:{min(int(t1[17:19])+15, 59):02d}'
        i2 = bisect.bisect_right(ts, t2)
        seg = frames[i0:i2]
        tr = track(seg)
        res = wins[wi][3]
        # 发布坐标按秒取最近一帧
        pose_map = {}
        for pt in poses[bisect.bisect_left(poseTs, t0):bisect.bisect_right(poseTs, t2)]:
            pose_map.setdefault(pt[0][:19], (pt[1], pt[2]))
        print(f'\n------ 窗口#{wi+1}{note}  精调点{t0[11:]} → sit_down_high{tS[11:]} → 趴下完成{t1[11:]} (+15s)  【{res}】------')
        # 电流检测事件(精调点 ~ 趴下完成后+15s)
        w_dets = [d for d in dets if t0 <= d[0] <= t2]
        if w_dets:
            ds = '  '.join(f"第{d[1]}次 {d[0][11:19]}" +
                           (f" (目标 x:{d[2]:+.3f}, y:{d[3]:+.3f})" if d[2] is not None else '')
                           for d in w_dets)
            print(f'电流检测: {ds}')
        print(f'{"时刻":<12}{"簇数":>4}  {"左柱(x,y)":>18}  {"右柱(x,y)":>18}  {"机器人定位(x,y)":>20}')
        shown = set()
        for t, L, R, n in tr:
            kk = t[:19]  # 每秒 1 帧
            if kk in shown: continue
            shown.add(kk)
            if tS[:19] <= kk < (tS[:17] + f'{int(tS[17:19])+1:02d}'):
                print(f'{"":<12}―――――― sit_down_high (Z降到0.1, 开始趴下) ――――――')
            elif t1[:19] <= kk < (t1[:17] + f'{int(t1[17:19])+1:02d}'):
                print(f'{"":<12}―――――― 趴下完成 (Z到-0.1, 开始趴桩测试) ――――――')
            for d in w_dets:
                if d[0][:19] <= kk < (d[0][:17] + f'{int(d[0][17:19])+1:02d}'):
                    tgt = f'目标 x:{d[2]:+.3f}, y:{d[3]:+.3f}' if d[2] is not None else '无目标位置日志'
                    print(f'{"":<12}―――――― 第{d[1]}次检测电流 ({tgt}) ――――――')
            P = pose_map.get(kk)
            print(f'{t[11:19]:<12}{n:>4}  {("(" + f"{L[0]:+.3f},{L[1]:+.3f}" + ")") if L else "--":>18}  {("(" + f"{R[0]:+.3f},{R[1]:+.3f}" + ")") if R else "--":>18}  {("(" + f"{P[0]:+.3f},{P[1]:+.3f}" + ")") if P else "--":>20}')

    # 失败(无电流)窗口单独放最前
    print('\n\n############ 一、失败(无电流)窗口明细 ############')
    for wi in range(len(wins)):
        if wins[wi][3].startswith('无电流'):
            detail(wi, ' ←无电流')

    # 全部窗口按时间顺序(失败的也包含并标注)
    print('\n\n############ 二、全部窗口按时间顺序 ############')
    for wi in range(len(wins)):
        note = ' ←无电流' if wins[wi][3].startswith('无电流') else ''
        detail(wi, note)

    sys.stdout.close()
    sys.stderr.write(f'完成: {os.path.abspath(out)}\n')

if __name__ == '__main__':
    main()
