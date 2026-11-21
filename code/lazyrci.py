# -*- coding: utf-8 -*-
# Exact separation of the rounded capacity inequalities (24) on demand (Section 5.3).
# Original file name in the authors' project: lazyrci.py
"""
rounded capacity inequalities（RCI）的按需分离。

原来的做法：建网络时枚举客户集合的全部 2^n 个子集，modelingDP_CUT 把右端项 >= 2 的
子集全部作为 capacityCons_ 约束一次性放进 RMP，定价与列系数也都遍历这张表。
18 客户时右端项 >= 2 的子集有 41738 个，24 客户时是 10901364 个，RMP 装不下，
定价与列系数的开销也随之爆炸。

现在的做法（cut-and-price 的标准写法）：
  1. RMP 初始不含任何 RCI；
  2. 某个节点上列生成收敛（定价找不到负检验数的列）后，对当前 LP 解做一次精确分离：
     对所有右端项 >= 2 的子集 C' 算 sum_{r: R_r ∩ C' ≠ ∅} λ_r，找出小于右端项的那些，
     按违反量从大到小取前 MAX_ADD 条加入 RMP（系数按现有全部列算好），再回到列生成；
  3. 既无改进列、又无被违反的割时，该节点的 LP 才算解完。

正确性：终止时的解满足全部 RCI（分离是精确的），且相对于"未加入的割对偶为 0"的完整
对偶向量，所有列的检验数都非负，于是它就是"预先放入全部 RCI"那个 LP 的最优解，界完全
相同；只是加入 RMP 的割少得多，每列只需对已加入的割算系数，定价里 Pi(u) 也只对已加入的
割求和。

子集族用位掩码存放（numpy 数组，24 客户 1090 万个子集约 55 MB），分离一次是对 1090 万个掩码做几次
按位与，不到一秒。约束命名沿用 capacityCons_{[c1, c2, ...]}，与驱动脚本里取对偶、算列
系数的写法完全一致；network.allSubset 在各节点上指向该节点已加入的割列表（node.rciSubsets），
驱动脚本里遍历 network.allSubset 的代码因此不必改。
"""
import re

import numpy as np

TOL = 1e-6        # 违反量阈值
MAX_ADD = 100     # 每轮最多加入的割数

_KEY_INTS = re.compile(r'\d+')
_FAMILY_CACHE = {}


def family(network):
    """所有右端项 >= 2 的客户子集：返回 (masks, rhs)，位序按 network.Customer 的下标。按 (客户, 需求, 容量) 缓存。"""
    cust = tuple(int(c) for c in network.Customer)
    dem = tuple(int(x) for x in network.demand)
    Q = int(max(network.vehicleCap))
    key = (cust, dem, Q)
    fam = _FAMILY_CACHE.get(key)
    if fam is not None:
        return fam
    n = len(cust)
    if n > 31:
        raise RuntimeError('客户数超过 31，位掩码需改为 uint64')
    N = 1 << n
    idx = np.arange(N, dtype=np.uint32)
    tot = np.zeros(N, dtype=np.int32)
    for j in range(n):
        tot[((idx >> np.uint32(j)) & np.uint32(1)).astype(bool)] += dem[j]
    sel = tot > Q                                   # ceil(d/Q) >= 2  <=>  d > Q
    masks = np.ascontiguousarray(idx[sel])
    rhs = ((tot[sel] + Q - 1) // Q).astype(np.int8)
    _FAMILY_CACHE[key] = (masks, rhs)      # 放在模块里而不是 network 上：建进程池时 network 要序列化给工作进程
    return masks, rhs


def family_size(network):
    return int(len(family(network)[0]))


def column_mask(entry, n):
    """列池条目 -> 客户位掩码。初始列是 0/1 列表；生成的列前 n 项是 (alpha, beta, gamma) 三元组。"""
    m = 0
    for k in range(n):
        v = entry[k]
        if isinstance(v, (tuple, list)):
            v = sum(v)
        if v:
            m |= 1 << k
    return m


def subset_of(mask, network):
    return [network.Customer[k] for k in range(len(network.Customer)) if (mask >> k) & 1]


def separate(node, network, max_add=MAX_ADD, tol=TOL):
    """对 node.RMP 的当前 LP 解做精确分离，把违反的 RCI 加入 RMP 与 node.rciSubsets。
    返回 (加入条数, 新增约束对象列表)。调用前 RMP 必须处于最优状态。"""
    masks, rhs = family(network)
    if len(masks) == 0:
        return 0, []
    n = len(network.Customer)
    rmp = node.RMP
    all_vars = rmp.getVars()
    col_masks = []
    for v in all_vars:
        entry = node.columnPool.get(v.VarName)
        if entry is None:
            raise RuntimeError('列池里没有变量 %s 的记录，无法分离' % v.VarName)
        col_masks.append(column_mask(entry, n))
    cover = np.zeros(len(masks), dtype=np.float64)
    for v, m in zip(all_vars, col_masks):
        x = v.X
        if x > tol and m:
            cover += x * ((masks & np.uint32(m)) != 0)
    viol = rhs - cover
    cand = np.flatnonzero(viol > tol)
    if cand.size == 0:
        return 0, []
    order = np.argsort(-viol[cand], kind='stable')
    cand = cand[order[:max_add]]
    added = []
    for i in cand:
        m = int(masks[i])
        S = subset_of(m, network)
        expr = 0
        for v, cm in zip(all_vars, col_masks):
            if cm & m:
                expr += v
        c = rmp.addConstr(expr >= int(rhs[i]), name='capacityCons_%s' % S)
        node.rciSubsets.append(S)
        added.append(c)
    rmp.update()
    return len(added), added


def rollback(node, cons):
    """撤回刚加入的割（受限主问题因它们变得不可行时用）。"""
    if not cons:
        return
    for c in cons:
        node.RMP.remove(c)
    del node.rciSubsets[len(node.rciSubsets) - len(cons):]
    node.RMP.update()
