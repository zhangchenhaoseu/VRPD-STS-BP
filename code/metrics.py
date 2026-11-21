# -*- coding: utf-8 -*-
# Run statistics recorder (iterations, pricing calls and times, bounds) and the wall-clock deadline used by the drivers.
# Original file name in the authors' project: metrics.py
# 分支定价运行统计记录器
# 用于回复 Reviewer 1 的 Comment 3 / 4 / 5：
#   C3 定价子问题的调用次数与逐次用时
#   C4 有效不等式的数量，以及根节点处对偶值非零（起作用）的条数
#   C5 根节点 LP 界、终止时的上下界与 gap
# 本模块只做记录，不改变任何算法逻辑。

import json
import math
import statistics as _st
import time
from collections import defaultdict

# 壁钟截止时刻，由各 branchAndPrice* 在开始时设置（见 patch_walltime.py）。
# 原来的判据是 (SPtimeCostInBP + RMPtimeCostInBP) <= timeLimit，累加的只是子问题
# 与主问题的纯计算时间，序列化、进程分发、列的构造都不计入，并行版尤其悬殊，
# 时限因此形同虚设。改为壁钟后，正文里写的 7200 s / 10800 s 才是它字面的意思。
DEADLINE = None


def expired():
    return DEADLINE is not None and time.perf_counter() > DEADLINE


def _san(v):
    """把 inf / nan 转成字符串，保证写出的 json 是严格合法的。"""
    if isinstance(v, float) or hasattr(v, 'dtype'):
        try:
            f = float(v)
        except (TypeError, ValueError):
            return v
        if math.isnan(f):
            return 'nan'
        if math.isinf(f):
            return 'inf' if f > 0 else '-inf'
        return f
    return v


class Recorder:
    def __init__(self):
        self.reset()

    def reset(self):
        self.d = defaultdict(int)          # cg_iters, bb_nodes, pricing_calls
        self.pricing_times = []            # 每一次定价子问题求解的纯计算用时
        self.root_lp = None                # 根节点列生成收敛后的 LP 目标值
        self.n_rci = None                  # 枚举出的 rounded capacity inequalities 总条数
        self.root_rci_active = None        # 根节点最优解处对偶值非零的条数

    def add_pricing(self, dt):
        self.d['pricing_calls'] += 1
        self.pricing_times.append(float(dt))

    def dump(self, path, extra=None):
        out = {k: _san(v) for k, v in self.d.items()}
        out['root_lp'] = _san(self.root_lp)
        out['n_rci_enumerated'] = _san(self.n_rci)
        out['n_rci_active_root'] = _san(self.root_rci_active)
        if self.pricing_times:
            out['pricing_time_total'] = _san(sum(self.pricing_times))
            out['pricing_time_mean'] = _san(_st.mean(self.pricing_times))
            out['pricing_time_max'] = _san(max(self.pricing_times))
            out['pricing_time_min'] = _san(min(self.pricing_times))
        if extra:
            for k, v in extra.items():
                out[k] = _san(v)
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(out, f, ensure_ascii=False, indent=2)
        print('[metrics]', path)
        print('[metrics]', out)


class _NoSolRMP:
    """占位对象，仅在时限内没有找到任何整数解时使用。"""
    ObjVal = float('inf')

    def getVars(self):
        return []


class NoSolution:
    """时限内未找到整数解时替代 incumbent，避免结果写出语句报错。"""

    def __init__(self):
        self.RMP = _NoSolRMP()
        self.columnPool = {}
        self.columnPool_SP = {}


REC = Recorder()
