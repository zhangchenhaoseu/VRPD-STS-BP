# -*- coding: utf-8 -*-
# Pricing dynamic program used in all reported runs (Section 5.4, Appendix D): one label per state, chronological extension, cached multiplier increments, optional state dominance (USE_DOMINANCE).
# Original file name in the authors' project: DPforSPP_fast.py
"""
定价子问题动态规划的加速实现（与原 DPforSPP_operation* 系列等价）。

三处加速，前两处严格等价，第三处基于 Property 1：

1) 有效不等式乘子增量的缓存。
   原实现在每一次状态转移里都要遍历 network.allSubset（|C|=12 时 4095 个子集）两遍，
   再做集合差与字符串拼键。注意 Φ(u) 只取决于掩码 u，且扩展时恒有 u ⊆ u_new，
   于是所需增量 = Pi(u_new) - Pi(u)，其中
       Pi(u) = Σ_{C' ∩ u ≠ ∅} π³_{C'}
   按掩码缓存即可，结果与原实现逐位相同。

2) 按时刻分桶存放状态。
   原实现每个时刻都用 [s for s in dp if s[1] == t] 扫描整张 dp 表，复杂度 O(T·|dp|)。

3) 状态支配规则（默认关闭，USE_DOMINANCE=False）。
   曾经的想法是：由于每个操作满足 endTime - startTime = operationTimeCost，
   且路径上的操作首尾相接、自时刻 0 出发，任一状态的累计成本
       cost(i, t, u) = t - Sum_{c in u} pi1_c - Pi(u)
   只取决于 (t, u)，因此同一 (节点, 掩码) 下到达更早者成本更低，晚到者可剪。
   成本恒等式本身是对的，但"晚到者可剪"要求早到者能够复制晚到者的一切后续，
   而这依赖于原地等待——等待操作产生的恰恰是同一 (节点, 掩码) 的更晚到达，
   一旦按上述规则剪掉，车就再也不能在原地停留，路径中"等两个时刻再接上某个
   只在该时刻出发的操作"这一类走法会被整体丢掉。在时变-SF 9 客户的真实操作集
   上，20 组随机对偶里有 6 组因此漏掉了最优列，进而使列生成提前收敛、根节点
   LP 界虚高，个别算例甚至出现下界高于上界。

   把原地不动的转移排除在剪枝之外即可修正（代码已按此写好），但修正后等待链
   仍占据绝大部分状态，实测只快 11%，不值得为此承担额外的正确性风险，故默认
   关闭。关闭后本模块与 DPforSPP_operationValidCut 逐次等价，仍保有上面两处
   加速带来的数十至数百倍收益。

用法：把驱动脚本里的
    import DPforSPP_operationValidCut as dp_SPPVC
换成
    import DPforSPP_fast as dp_SPPVC
其余调用方式不变。并行版对应 shortest_path_operations_parallel。
"""

import math
import re  # [lazy]
import time
from collections import defaultdict

_LAZY_INTS = re.compile(r"\d+")  # [lazy]
USE_DOMINANCE = False         # 默认关闭，理由见上面第 3 条
LAST_STATS = {}               # 最近一次调用的统计量，便于记录到 metrics


def _prepare(operations, network):
    """预处理：为每个操作算好客户掩码，并按 (startID, startTime) 分组。"""
    cidx = {c: i for i, c in enumerate(network.Customer)}
    groups = defaultdict(list)
    for op in operations.values():
        mask = 0
        ok = True
        for c in op.ServiceID:
            j = cidx.get(c)
            if j is None:                 # 与原实现一致：不在客户表中的编号忽略
                continue
            bit = 1 << j
            if mask & bit:                # 同一操作内重复服务同一客户，原实现判为不可行
                ok = False
                break
            mask |= bit
        if ok:
            groups[(op.startID, op.startTime)].append(
                (op, mask, op.ServiceDemand, op.ReducedCost))
    return cidx, groups


def _make_pi(network, cidx, capacityPi_DICT):
    """返回 Pi(u)：与掩码 u 所含客户相交的全部 RCI 子集的乘子之和，按需缓存。"""
    if not capacityPi_DICT:
        return lambda u: 0.0
    subsets = []  # [lazy] 只遍历已加入 RMP 的割（capacityPi_DICT 的键就是这些割的客户子集），
    for key, p in capacityPi_DICT.items():   # 插入次序与原来遍历 allSubset 的次序相同，求和结果逐位一致
        if not p:
            continue
        m = 0
        for c in _LAZY_INTS.findall(key):
            j = cidx.get(int(c))
            if j is not None:
                m |= 1 << j
        if m:
            subsets.append((m, p))
    cache = {0: 0.0}

    def total_pi(u):
        v = cache.get(u)
        if v is None:
            v = 0.0
            for m, p in subsets:
                if m & u:
                    v += p
            cache[u] = v
        return v

    return total_pi


BONUS_LIMIT = 1e6      # 超过这个量级的检验数，判定为分支的大数调整 adjustCoff


def dominance_is_safe(operations, endS, tol=1e-6):
    """
    判断本次调用能否使用支配规则。

    支配的前提是 cost(i,t,u) = t - Π(u) 只取决于 (t,u)。分支会破坏它：
    右子节点用大数 adjustCoff 压低某个指定操作的检验数，分支约束的对偶值
    也只加在特定的操作键上，于是同一 (节点, 时刻, 掩码) 可以有不同成本，
    早到不再必然更省。此时必须退回逐状态搜索。

    检验方式：对每个操作算 delta = ReducedCost - (endTime - startTime)。
    identity 成立时 delta 恒等于 -sumOfCustomerPi，因而只取决于该操作服务的
    客户集合；某个操作被单独调整过，它就会和同客户集合的其他操作对不上。
    终点车场的零成本等待操作单独排除，它们不影响结论。
    """
    by_service = {}
    for op in operations.values():
        if op.startID == endS and op.endID == endS:
            continue
        if abs(op.ReducedCost) > BONUS_LIMIT:      # mustUse 的大数调整
            return False
        key = tuple(sorted(op.ServiceID))
        d = op.ReducedCost - (op.endTime - op.startTime)
        if not key and abs(d) > tol:               # 无客户操作的基准必须为 0
            return False
        prev = by_service.get(key)
        if prev is None:
            by_service[key] = d
        elif abs(prev - d) > tol:                  # 同客户集合内部不一致
            return False
    return True


def _solve(operations, network, truckGroupCapacity, capacityPi_DICT, use_dominance,
           skip_guard=False):
    if not operations:
        return None, []

    dominance_blocked = False
    if use_dominance and not skip_guard and not dominance_is_safe(operations, network.D):
        use_dominance = False
        dominance_blocked = True

    cidx, groups = _prepare(operations, network)
    total_pi = _make_pi(network, cidx, capacityPi_DICT)
    startS, endS = network.O, network.D
    max_time = max(op.endTime for op in operations.values())

    dp = {}                                   # state -> (cost, 前驱, 到达该状态所用的操作)
    by_time = defaultdict(list)               # 时刻 -> 该时刻的状态列表
    init = (startS, 0, 0, 0)
    dp[init] = (0.0, None, None)
    by_time[0].append(init)

    best_time = {(startS, 0): 0} if use_dominance else None   # (节点, 掩码) -> 最早到达时刻

    result, best_state = math.inf, None
    n_pop = n_ext = 0

    for t in range(0, max_time + 1):
        bucket = by_time.get(t)
        if not bucket:
            continue
        k = 0
        while k < len(bucket):                # 用下标遍历，容忍遍历中追加
            state = bucket[k]
            k += 1
            currentS, _t, used, demand = state
            cost = dp[state][0]
            n_pop += 1

            if currentS == endS and cost < result:
                result, best_state = cost, state

            ops_here = groups.get((currentS, t))
            if not ops_here:
                continue
            pi_used = total_pi(used)
            for op, mask, dem, rc in ops_here:
                if used & mask:               # 客户重复服务
                    continue
                new_demand = demand + dem
                if new_demand > truckGroupCapacity:
                    continue
                new_used = used | mask
                new_cost = cost + rc - (total_pi(new_used) - pi_used)
                end_t = op.endTime
                if use_dominance and not (op.endID == currentS and new_used == used):
                    bt = best_time.get((op.endID, new_used))
                    if bt is not None and bt <= end_t:
                        continue
                    best_time[(op.endID, new_used)] = end_t
                new_state = (op.endID, end_t, new_used, new_demand)
                old = dp.get(new_state)
                if old is None:
                    dp[new_state] = (new_cost, state, op)
                    by_time[end_t].append(new_state)
                    n_ext += 1
                elif new_cost < old[0]:
                    dp[new_state] = (new_cost, state, op)

    for state, entry in dp.items():
        if state[0] == endS and entry[0] < result:
            result, best_state = entry[0], state

    best_path = []
    if best_state is not None:
        cur = best_state
        while cur is not None:
            entry = dp.get(cur)
            if entry is None:
                break
            if entry[2] is not None:
                best_path.append(entry[2])
            cur = entry[1]
        best_path.reverse()

    LAST_STATS.clear()
    LAST_STATS.update(states_created=len(dp), states_expanded=n_pop,
                      transitions=n_ext, dominance=bool(use_dominance),
                      dominance_blocked=dominance_blocked)

    return (result, best_path) if result < math.inf else (None, [])


def shortest_path_operations(operations, network, truckGroupCapacity,
                             capacityPi_DICT=None, use_dominance=None,
                             skip_guard=False):
    """对应 DPforSPP_operationValidCut.shortest_path_operations（含有效不等式）。

    skip_guard 仅供对照测试使用，正常调用不要打开。
    """
    if use_dominance is None:
        use_dominance = USE_DOMINANCE
    return _solve(operations, network, truckGroupCapacity, capacityPi_DICT,
                  use_dominance, skip_guard)


def shortest_path_operations_nocut(operations, network, truckGroupCapacity,
                                   use_dominance=None):
    """对应 DPforSPP_operation.shortest_path_operations（不含有效不等式）。"""
    if use_dominance is None:
        use_dominance = USE_DOMINANCE
    return _solve(operations, network, truckGroupCapacity, None, use_dominance)


def shortest_path_operations_parallel(input_data):
    """对应 DPforSPP_operationValidCut_parallel.shortest_path_operations。"""
    operations, network, truckGroupCapacity, capacityPi_DICT = input_data
    t0 = time.perf_counter()
    cost, path = _solve(operations, network, truckGroupCapacity,
                        capacityPi_DICT, USE_DOMINANCE)
    dt = time.perf_counter() - t0
    return (cost, path, dt) if cost is not None else (None, [], dt)
