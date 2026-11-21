# -*- coding: utf-8 -*-
# Faster construction of the same operation set (byte-identical iniOperation.txt); used to build the operation files of the Sioux Falls instances with the width-15 time windows.
# Original file name in the authors' project: constructOperation_fast.py (last edited 2024/12/12)
# ======================================================================
# constructOperation_fast.py
# 与 constructOperation.py 输出完全相同（iniOperation.txt 逐字节一致，已在原时间窗的 SF13 和扩展
# 时间窗后的 SF10、时变 SF10、SF13、SF18、时变 SF18、SF21 上核对）的加速版本，2026-09-09 用它生成了
# 扩展时间窗后 1 到 24 客户共 48 个 SF 算例的 iniOperation.txt（新窗 SF24 共 148905 条操作，构建 258 秒）。
# 接口不变：FEoperationFunction(network, Truckcapacity, UAVcapacity, UAVendurance)。
# 想在驱动脚本里直接用，把 import constructOperation as cO 换成 import constructOperation_fast as cO 即可。
#
# 改动只在 E 类操作的搜索与遍历方式，不改任何规则：
#   1. shortest_path_E 的标签 (t - startT) + popcount(mask)*(-1e6) 只由到达时刻和已服务客户数决定，
#      与走法无关，原 SPFA 实际是 (节点, 时刻, 客户掩码) 上的可达性搜索；改为按时间分层一次推进，
#      同时给出到全部候选会合点的最早到达时刻（shortest_path_E_multi），不必对每个终点各搜一遍。
#   2. 搜索只推进到 originT + (续航 - 2)：建立 E 类操作要求卡车用时 <= UAVendurance - 2，更晚的结果用不上。
#   3. 只遍历真正存在架次的 (起点, 出发时刻, 终点)，并按 (起点, 出发时刻) 分组：卡车能否服务完
#      truckSet 与终点无关，走不通位图与缓存向量按组只查一次；本方案产生的操作先收进缓冲区，
#      再按原三重循环 (起点序, 终点序, 出发时刻) 的次序写入 Operation_Dict，写入先后与比较覆盖
#      的语义都与原实现一致，文件行序也一样。
#   4. 缓存按「卡车客户集合大小」分段：只保留本段走得通的结果和上一段走不通的位图（子集走不通
#      则超集必走不通），内存不再随枚举量线性增长。
#   5. 某一段（卡车客户集合大小 k）没有任何走得通的 (集合, 起点, 出发时刻) 时，更大的集合必然
#      也走不通（超集在任一节点的最早完成时刻不早于子集），E 类枚举就此结束，结果不变。
#      实测各 SF 算例都在 k = 4 或 5 结束，18 客户只需枚举 22 万个子集里的 1.2 万个。
#   6. 客户子集改为惰性枚举（generate_combinations_lazy，次序与 generate_combinations 完全相同）：
#      22 客户以上把全部可行子集放进列表会占 4 GB 以上内存，进程被系统杀掉。
#   7. numpy 标量一律转成内置 int 再做位运算与拼字符串（np.int64 下 1 << 340 会溢出成 0，
#      f"{[np.int64(30)]}" 会写成 [np.int64(30)]）。
# ======================================================================

# 注意这里面Candidate_rendezvous，两个路网是不同的！！！！要更换！！

# 增加了剪枝（优超关系）、缓存
# 根据网络（道路节点、客户节点）、无人机续航、乘子（客户约束乘子、车辆约束乘子、加强cut乘子、分支乘子），建立与之对应的所有可行的、有效操作（可行&最优）
import os, sys, math
from repo_paths import DATA_DIR, RESULT_DIR  # puts code/ and code/networks/ on sys.path
import numpy as np
import heapq
from collections import defaultdict,deque
import math

# [release] Payload-dependent endurance while the drone waits for the truck.
# The sortie file (step 4 of the data generation) already applies the payload term F(d) = d to the
# flight-and-wait span encoded in each sortie: flyIn + flyOut <= f - sL - sR - 2*sS - d. When a sortie
# is turned into a D- or E-type operation below, the drone may additionally hover at the recovery node
# until the truck arrives; in the original code this extra waiting is bounded only by the payload-free
# endurance, Truck_SPvalue <= UAVendurance - 2 (= f - sL - sR with sL = sR = 1). Setting the switch to
# True applies rule (3) of the paper to the whole launch-service-recovery cycle instead,
# Truck_SPvalue <= UAVendurance - 2 - d. All results reported in the paper were obtained with False;
# the operation files (iniOperation.txt) have to be rebuilt after changing it. See README, section
# "Differences between the released code and the paper".
CHARGE_PAYLOAD_WHILE_HOVERING = False

# [release] Candidate rendezvous (launch/recovery) nodes of the instance. The 9-node network uses
# [1, 2, 3, 4, 5, 6, 7, 8, 9], the Sioux Falls instances of Sections 6.2 and 6.3 use the twelve nodes
# below, and the rendezvous-distribution instances of Section 6.4.3 use the six-node sets of Table 13.
# Sorties launched or recovered at a node that is not in this list never become operations, so the
# list has to match the instance (FEoperationFunction prints a warning when it does not).
CANDIDATE_RENDEZVOUS = [1, 2, 5, 8, 10, 12, 13, 14, 17, 18, 20, 22]


_ADJ_D = {}   # id(Bx) -> 最短路 D 用的邻接表
_ADJ_E = {}   # id(Bx) -> 最短路 E 用的邻接表


class Operation:
    def __init__(self):
        self.startID = None  # 这个操作的起点节点索引（一定是道路节点或起终点车场）
        self.startTime = None  # 这个操作的开始时刻
        self.endID = None  # 这个操作的终点节点索引（一定是道路节点或终点车场）
        self.endTime = None  # 这个操作的结束时刻
        self.VehicleId = None  # 这个操作的车辆id
        self.operationName = None  # 这个操作的名字（与字典中不同）
        self.key = None  # 这个操作的键值（与字典中的键相同）

        self.same = True  # 这个判断在这个操作中，卡车和无人机是否走了一样的弧，默认是True

        self.droneTimeCost = None  # 这个操作中，无人机弧所花费的时间之和（不考虑乘子）
        self.truckTimeCost = None  # 这个操作中，卡车弧所花费的时间之和（不考虑乘子）
        self.operationTimeCost = None  # 这个操作对应的所花费的成本（操作的用时= max(self.droneTimeCost,self.truckTimeCost) ，不考虑乘子）

        self.droneServiceID = []  # 这个操作中无人机所服务的客户点（客户的虚拟节点），包括分离和合体，在self.same = Fales的情况下不可能为空
        self.truckServiceID = []  # 这个操作中卡车所服务的客户点（客户的虚拟节点），包括分离和合体，可能为空
        self.ServiceID = []  # 该操作所有服务的客户
        self.ServiceDemand = None  # 该操作所服务的总需求量

        self.sumOfCustomerPi = None  # 操作的客户约束乘子之和（取决于这个操作经过了哪些客户，将对应客户的乘子求和即可，此处为负数）这个在建立操作时用不到
        self.ReducedCost = None  # 这个操作对应的检验数（= self.operationTimeCose + self.sumOfCustomerPi）这个在建立操作时用不到


def precompute_dist_matrix(Bx, candidate_nodes, T_max):
    dist_matrix = defaultdict(dict)
    for i in candidate_nodes:
        for j in candidate_nodes:
            result = shortest_path_D(Bx, i, 0, j, T_max)
            dist_matrix[i][j] = result[0] if result else math.inf
    return dist_matrix

# 修改generate_combinations函数（优化组合生成）
def generate_combinations(customer, demand, TruckCapacity, UAVCapacity):
    """优化后的组合生成函数"""
    """例如[[(['c2'], 'c1')], [(['c3'], 'c1')], [(['c4'], 'c1'), (['c1'], 'c4')], [(['c2'], 'c4')], [(['c3'], 'c4')], [(['c2', 'c4'], 'c1'), (['c1', 'c2'], 'c4')], [(['c3', 'c4'], 'c1'), (['c1', 'c3'], 'c4')]]"""
    """ result[0] = [(['c2'], 'c1')]
        result[1] = [(['c3'], 'c1')]
        result[2] = [(['c4'], 'c1'), (['c1'], 'c4')]
        result[3] = [(['c2'], 'c4')]
        result[4] = [(['c3'], 'c4')]
        result[5] = [(['c2', 'c4'], 'c1'), (['c1', 'c2'], 'c4')]
        result[6] = [(['c3', 'c4'], 'c1'), (['c1', 'c3'], 'c4')]"""
    result = []
    # 预计算客户需求
    customer_demand = [(c, demand[i]) for i, c in enumerate(customer)]

    # 按需求排序便于剪枝
    customer_demand.sort(key=lambda x: x[1])

    # 迭代生成有效组合
    for r in range(2, len(customer) + 1):
        # 生成组合时进行剪枝
        stack = [(tuple(), 0, 0)]  # (当前组合, 总需求, 起始索引)

        while stack:
            current, total, idx = stack.pop()
            if len(current) == r:
                # 生成卡车和无人机的分配方案
                valid_assignments = []
                for c in current:
                    uav_demand = next((d for cust, d in customer_demand if cust == c), 0)
                    if uav_demand > UAVCapacity:
                        continue
                    truck_clients = [cust for cust in current if cust != c]
                    truck_demand = sum(d for cust, d in customer_demand if cust in truck_clients)
                    if truck_demand <= TruckCapacity:
                        valid_assignments.append((truck_clients, c))
                if valid_assignments:
                    result.append(valid_assignments)
                continue

            # 剪枝：剩余元素是否可能满足需求
            remaining = r - len(current)
            for i in range(idx, len(customer_demand) - remaining + 1):
                cust, d = customer_demand[i]
                new_total = total + d
                # 容量剪枝
                if new_total > TruckCapacity + UAVCapacity:
                    continue
                # 扩展栈（后进先出优化内存）
                stack.append((current + (cust,), new_total, i + 1))

    return result


def generate_combinations_lazy(customer, demand, TruckCapacity, UAVCapacity):
    """generate_combinations 的惰性版本：同样的深度优先枚举、同样的先后次序，逐个 yield。
    24 客户时可行子集有几百万个，整表放进内存会把进程撑爆（22 客户即被 OOM 杀掉）；
    而 E 类枚举在卡车客户集合大小 4 左右就会因无可行结果提前结束，根本用不到后面的子集。"""
    customer_demand = [(c, demand[i]) for i, c in enumerate(customer)]
    customer_demand.sort(key=lambda x: x[1])
    for r in range(2, len(customer) + 1):
        stack = [(tuple(), 0, 0)]
        while stack:
            current, total, idx = stack.pop()
            if len(current) == r:
                valid_assignments = []
                for c in current:
                    uav_demand = next((d for cust, d in customer_demand if cust == c), 0)
                    if uav_demand > UAVCapacity:
                        continue
                    truck_clients = [cust for cust in current if cust != c]
                    truck_demand = sum(d for cust, d in customer_demand if cust in truck_clients)
                    if truck_demand <= TruckCapacity:
                        valid_assignments.append((truck_clients, c))
                if valid_assignments:
                    yield valid_assignments
                continue
            remaining = r - len(current)
            for i in range(idx, len(customer_demand) - remaining + 1):
                cust, d = customer_demand[i]
                new_total = total + d
                if new_total > TruckCapacity + UAVCapacity:
                    continue
                stack.append((current + (cust,), new_total, i + 1))


def shortest_path_D(Bx, startS, startT, endS, T_max):  # 添加最大时间T_max作为参数
    """
    在时空网络中找到从起点到终点的最短路径，目标是最早到达终点。
    如果到达时间大于给定的最大时间T，则返回None。
    参数：
        Bx: list，时空弧集合，格式为 [(i, j, t, s), ...]，表示从节点 i 在时间 t 出发，到节点 j 在时间 s 到达的弧。
        startS: int，起点节点。
        startT: int，起点出发时间。
        endS: int，终点节点。
        T: int，最大允许的到达时间。
    返回：
        最短路径的长度和路径。如果不可达，或者到达时间超过最大时间T，返回 None。
    """
    # 初始化优先队列和禁忌表
    pq = []  # 优先队列，存储 (当前路径长度, 当前节点, 当前时间, 路径)
    heapq.heappush(pq, (0, startS, startT, [(startS, startT)]))  # 初始化路径长度为0，开始节点和时间为起点
    # 初始化访问记录
    visited = set()
    # 构建时空弧的邻接表（Bx 不变，只建一次）
    _k = id(Bx)
    adjacency = _ADJ_D.get(_k)
    if adjacency is None:
        adjacency = {}
        for i, j, t, s in Bx:
            if (i, t) not in adjacency:
                adjacency[(i, t)] = []
            adjacency[(i, t)].append((j, s))
        _ADJ_D[_k] = adjacency
    while pq:
        cost, current_node, current_time, path = heapq.heappop(pq)
        # 如果到达目标节点，检查到达时间是否超出最大允许时间
        if current_node == endS:
            if current_time > T_max:  # 如果结束时间大于最大时间T，返回None
                return None
            return cost, path
        # 避免重复状态
        state = (current_node, current_time)
        if state in visited:
            continue
        visited.add(state)
        # 遍历当前节点的所有邻居
        for next_node, next_time in adjacency.get((current_node, current_time), []):
            # 计算新路径的成本，考虑时间差
            new_cost = cost + (next_time - current_time)
            # 将新的路径信息加入优先队列
            heapq.heappush(pq, (new_cost, next_node, next_time, path + [(next_node, next_time)]))
    # 如果队列为空仍未找到路径，则返回 None
    return None

# SPFA算法：使用双端队列处理节点松弛，优化负权边的处理效率。
# 状态压缩：用位掩码记录客户访问状态，减少内存占用。
# 提前剪枝：路径成本超过已知最小值或时间超限时，停止扩展。
# 邻接表预处理：将时空弧预处理为邻接表，减少重复计算。
# 终点检查优化：在松弛过程中直接检查是否到达终点并访问全部客户，避免最后遍历。
def shortest_path_E(Bx, startS, startT, endS, customerSubsets, T_max):  # 改用SPFA（队列优化的Bellman-Ford）算法结合状态压缩，有效处理负权并减少计算量。
    if not customerSubsets:
        result = shortest_path_D(Bx, startS, startT, endS, T_max)
        return result[0] if result else None

    ATTRACT_FACTOR = -1000000  # 客户吸引力因子
    num_customers = len(customerSubsets)
    customer_set = set(customerSubsets)
    mask = (1 << num_customers) - 1  # 所有客户都需访问的掩码

    # 构建时空邻接表（Bx 不变，只建一次）
    _k = id(Bx)
    adjacency = _ADJ_E.get(_k)
    if adjacency is None:
        adjacency = defaultdict(list)
        for i, j, t, s in Bx:
            adjacency[(i, t)].append((j, s, (s - t)))  # 存储节点、到达时间、时间消耗
        _ADJ_E[_k] = adjacency
    _cidx = {c: k for k, c in enumerate(customerSubsets)}

    # SPFA初始化
    dist = defaultdict(lambda: defaultdict(lambda: math.inf))
    queue = deque()
    in_queue = defaultdict(lambda: defaultdict(bool))

    # 初始状态：起点、时间、未访问任何客户
    initial_used = 0
    dist[startS][(startT, initial_used)] = 0
    queue.append((startS, startT, initial_used))
    in_queue[startS][(startT, initial_used)] = True

    min_cost = math.inf

    while queue:
        u, t_u, used = queue.popleft()
        in_queue[u][(t_u, used)] = False

        # 提前剪枝：若当前路径已超过最小成本或时间限制
        current_cost = dist[u][(t_u, used)]
        if current_cost >= min_cost or t_u > T_max:
            continue

        # 遍历所有相邻节点
        for (v, t_v, delta_t) in adjacency.get((u, t_u), []):
            new_used = used
            # 检查是否为需要访问的客户节点
            if v in customer_set:
                idx = _cidx[v]
                if not (used & (1 << idx)):
                    new_used = used | (1 << idx)
                    delta_t += ATTRACT_FACTOR  # 应用吸引力因子

            new_cost = current_cost + delta_t

            # 更新最短路径
            if new_cost < dist[v][(t_v, new_used)]:
                dist[v][(t_v, new_used)] = new_cost

                # 到达终点且访问全部客户，更新最小值
                if v == endS and new_used == mask:
                    effective_cost = new_cost - len(customerSubsets) * ATTRACT_FACTOR
                    if effective_cost <= (T_max - startT):
                        min_cost = min(min_cost, effective_cost)

                # 若节点不在队列中，则加入队列
                if not in_queue[v][(t_v, new_used)]:
                    queue.append((v, t_v, new_used))
                    in_queue[v][(t_v, new_used)] = True

    return min_cost if min_cost != math.inf else None



_ADJ_M = {}   # id(Bx) -> 多终点最短路用的邻接表


def shortest_path_E_multi(Bx, startS, startT, truckSet, T_max):
    """一次搜索给出「从 (startS, startT) 出发、把 truckSet 里的客户全部服务完之后，
    最早能在各个节点出现的时刻」。

    原来的 shortest_path_E 是单终点的：标签 new_cost 里带一个 -1e6 的客户吸引力项，
    但沿路径累加的 delta_t 是望远镜和，等于 t_v - startT，吸引力项又只取决于已服务
    客户的个数，所以 dist[v][(t_v, used)] 恒等于 (t_v - startT) + popcount(used)*A，
    与走法无关。也就是说那套 SPFA 实际做的是状态 (节点, 时刻, 客户掩码) 上的可达性
    搜索，返回的 min_cost 就是「服务完全部客户后到达终点的最早时刻减去 startT」。
    既然如此，一次搜索本来就能同时给出所有终点的答案，不必对 12 个候选会合点各搜一遍。

    另外原版对 used == 0 的状态有一条 current_cost >= min_cost 的剪枝，它只丢弃
    「注定不可能更优」的状态，不影响返回值；多终点下 min_cost 不再唯一，故去掉。

    Bx 中的弧都严格向前推进时间（唯一的零时长弧是 O->D 的状态 0 弧，不在 Bx 里），
    所以按时间分层推进即可，不需要优先队列。
    """
    _k = id(Bx)
    adjacency = _ADJ_M.get(_k)
    if adjacency is None:
        adjacency = {}
        for i, j, t, s in Bx:
            adjacency.setdefault((i, t), []).append((j, s))
        _ADJ_M[_k] = adjacency

    bit = {}
    for _k2, _c in enumerate(truckSet):
        bit[_c] = 1 << _k2
    full = (1 << len(truckSet)) - 1

    best = {}                                  # 节点 -> 服务完全部客户后最早到达的时刻
    layers = [None] * (T_max + 1)
    layers[startT] = {(startS, 0)}
    for t in range(startT, T_max + 1):
        cur = layers[t]
        if not cur:
            continue
        layers[t] = None                       # 该层处理完即释放
        for u, m in cur:
            if m == full and u not in best:
                best[u] = t
            for v, s in adjacency.get((u, t), ()):
                if s > T_max:
                    continue
                nm = m | bit.get(v, 0)
                lay = layers[s]
                if lay is None:
                    lay = layers[s] = set()
                lay.add((v, nm))
    return best


def FEoperationFunction(network,Truckcapacity, UAVcapacity, UAVendurance):  # 输入网络、无人机续航、无人机容量，建立可行的有效操作集合，后续需要对乘子进行调整
    # 构建Bx集合，用于D、E类操作构建
    Bx = []  # list，状态为1，1的时空弧集合，格式为 [(i, j, t, s), ...]，
    for vehicleArc in network.stsVehicleArcList:  # 卡车的时空状态弧，这里只用w=w’=1的时空状态弧
        if vehicleArc.from_state == 1 and vehicleArc.to_state == 1:
            Bx.append((vehicleArc.from_space,vehicleArc.to_space,vehicleArc.from_time,vehicleArc.to_time))

    # 预处理无人机架次到字典：键为(起点,出发时间,终点,客户)
    sortie_dict = {}
    for sortie in network.PotentialSortiesList:
        key = (sortie.start_space, sortie.start_time, sortie.end_space, sortie.delivery_space)
        if key not in sortie_dict or (sortie.end_time - sortie.start_time) < (sortie_dict[key].end_time - sortie_dict[key].start_time):
            sortie_dict[key] = sortie

    print("===>开始构建操作<===")
    full_list = list(range(network.O, network.D+1))
    transNetNode = list(range(network.O+1, network.D-len(network.Customer)))  # 底层运输网络节点集合
    Candidate_rendezvous = CANDIDATE_RENDEZVOUS  # 9节点候选会合点[1,2,3,4,5,6,7,8,9]、SF网络候选会合点[1,2,5,8,10,12,13,14,17,18,20,22]
    # [release] warn when the sortie file uses launch/recovery nodes that are not candidates here
    _sortie_nodes = sorted({s.start_space for s in network.PotentialSortiesList} | {s.end_space for s in network.PotentialSortiesList})
    _missing = [n for n in _sortie_nodes if n not in Candidate_rendezvous]
    if _missing:
        print(f"WARNING: Potential_sorties.txt uses launch/recovery nodes {_missing} that are not in CANDIDATE_RENDEZVOUS; "
              f"sorties at these nodes will not become operations.")
    STtransNetNode = []  # 以全连接形式存放所有底层运输网络节点集合对应的时空节点
    for spaceID in transNetNode:
        for timeID in network.T[2:-2]:  # D类操作计算卡车最短路的时候从第3个时间切片开始【0，1，2，..】即2开始，到倒数第3个结束
            STtransNetNode.append((spaceID,timeID))
    # 建立时空节点之间的邻接矩阵
    size = len(STtransNetNode)
    _st_index = {_p: _k for _k, _p in enumerate(STtransNetNode)}  # 时空节点 -> 行列号，替代 list.index 的线性扫描
    _transSet = set(transNetNode)
    _Tmid = set(network.T[2:-2])
    matrix = np.inf * np.ones((size, size))  # 默认标签是较大值
    for i in range(0, size):
        matrix[i][i] = np.inf  # 不存在瞬间移动
    for vehicleArc in network.stsVehicleArcList:  # 卡车的时空状态弧，这里只用w=w’=1的时空状态弧
        if vehicleArc.from_state == 1 and vehicleArc.to_state == 1 and vehicleArc.from_space in _transSet and vehicleArc.to_space in _transSet and vehicleArc.from_time in _Tmid and vehicleArc.to_time in _Tmid:  # 分离之后的卡车弧
            fromSpace = vehicleArc.from_space
            fromTime = vehicleArc.from_time
            toSpace = vehicleArc.to_space
            toTime = vehicleArc.to_time
            rowIndex = _st_index[(fromSpace, fromTime)]  # 这个时空起点对应距离矩阵的第几行
            colIndex = _st_index[(toSpace, toTime)]  # 这个时空起点对应距离矩阵的第几列
            matrix[rowIndex][colIndex] = toTime - fromTime  # 这里邻接矩阵成本就是弧的时间
    # print(matrix)
    # print(matrix[0])
    # A、B、C 三类操作原先都是在 (i, j, t) 的循环里逐个扫描整张卡车弧表，客户数一大就是
    # 几亿次比较。这里先把弧表按 (i, j, t) 与 (i, t) 建成索引，循环里直接取。字典按弧表
    # 顺序覆盖写入，即"后出现者胜"，与原来扫完整表、最后一次匹配留下的结果一致；
    # _by_ft 按原顺序追加，C 类的两层遍历因此逐条对应。
    _by_ijt = {}
    _by_ft = {}
    for _a in network.stsVehicleArcList:
        _by_ijt[(_a.from_space, _a.to_space, _a.from_time)] = _a
        _by_ft.setdefault((_a.from_space, _a.from_time), []).append(_a)
    Operation_Dict = {}
    """---------------第A类操作：出车&入库操作---------------（该操作访问的客户为空集[]）"""
    # (1-1) 出车操作
    for i in [network.O]:  # 出车操作起点i可以是：起点车场O
        for j in [network.O, network.D, full_list[1]]: # 出车操作的终点j可以是：起点车场O、终点车场D、第一个道路节点（索引一般就是1）
            for t in network.T:  # 出车操作可选择的时间点
                # 什么样情况才会建立这个操作呢？————对于任意的卡车和无人机的、在t时刻从i点出发前往j节点的合体弧，其终点时刻小于等于max（network.T）时，才会建立否则没意义
                for vehicleArc in ([_by_ijt[(i, j, t)]] if (i, j, t) in _by_ijt else []):  # 卡车弧,只要是构建出来的都是存在的，而且是邻接的（改为按 (i,j,t) 取索引，等价于原来扫全表留下的最后一条）
                    T_from_space = vehicleArc.from_space  # i
                    T_to_space = vehicleArc.to_space  # j
                    T_from_time = vehicleArc.from_time  # t
                    T_to_time = vehicleArc.to_time  # s
                    T_from_state = vehicleArc.from_state  # w
                    T_to_state = vehicleArc.to_state  # w_
                    T_cost = vehicleArc.cost  # 弧成本，其实就是时间
                    T_arcType = vehicleArc.arcType
                    if i == T_from_space and j == T_to_space and t == T_from_time:
                        operation = Operation()
                        operation.startID = i  # 操作起点
                        operation.startTime = t  # 操作开始时间
                        operation.endID = j  # 操作终点
                        operation.endTime = T_to_time  # 操作结束时间
                        operation.VehicleId = None
                        operation.operationName = f"A_{[]}_{i}_{j}_{operation.startTime}_{operation.endTime}"
                        operation.key = f"A_{[]}_{i}_{j}_{t}"
                        operation.same = True  # 肯定是同一个
                        operation.droneTimeCost = T_cost
                        operation.truckTimeCost = T_cost
                        operation.operationTimeCost = T_cost
                        operation.droneServiceID = []  # 出车操作无客户服务
                        operation.truckServiceID = []  # 出车操作无客户服务
                        operation.ServiceID = []  # 该操作无客户服务
                        operation.ServiceDemand = 0  # 该操作所服务的客户需求量
                        operation.sumOfCustomerPi = 0
                        operation.ReducedCost = operation.operationTimeCost-operation.sumOfCustomerPi
                        Operation_Dict[f"A_{[]}_{i}_{j}_{t}"] = operation
    # (1-2) 入库操作
    for i in [network.D, full_list[1]]:  # 出车操作起点i可以是：终点车场D、第一个道路节点（索引一般就是1）
        for j in [network.D]:  # 出车操作的终点j可以是：终点车场D
            for t in network.T:  # 出车操作可选择的时间点
                # 什么样情况才会建立这个操作呢？————对于任意的卡车和无人机的、在t时刻从i点出发前往j节点的合体弧，其终点时刻小于等于max（network.T）时，才会建立否则没意义
                for vehicleArc in ([_by_ijt[(i, j, t)]] if (i, j, t) in _by_ijt else []):  # 卡车弧,只要是构建出来的都是存在的，而且是邻接的（改为按 (i,j,t) 取索引，等价于原来扫全表留下的最后一条）
                    T_from_space = vehicleArc.from_space  # i
                    T_to_space = vehicleArc.to_space  # j
                    T_from_time = vehicleArc.from_time  # t
                    T_to_time = vehicleArc.to_time  # s
                    T_from_state = vehicleArc.from_state  # w
                    T_to_state = vehicleArc.to_state  # w_
                    T_cost = vehicleArc.cost  # 弧成本，其实就是时间（终点位置已经设置为0）
                    T_arcType = vehicleArc.arcType
                    if i == T_from_space and j == T_to_space and t == T_from_time:
                        operation = Operation()
                        operation.startID = i  # 操作起点
                        operation.startTime = t  # 操作开始时间
                        operation.endID = j  # 操作终点
                        operation.endTime = T_to_time  # 操作结束时间
                        operation.VehicleId = None
                        operation.operationName = f"A_{[]}_{i}_{j}_{operation.startTime}_{operation.endTime}"
                        operation.key = f"A_{[]}_{i}_{j}_{t}"
                        operation.same = True  # 肯定是同一个
                        operation.droneTimeCost = T_cost
                        operation.truckTimeCost = T_cost
                        operation.operationTimeCost = T_cost
                        operation.droneServiceID = []  # 入库操作无客户服务
                        operation.truckServiceID = []  # 入库操作无客户服务
                        operation.ServiceID = []  # 该操作无客户服务
                        operation.ServiceDemand = 0  # 该操作所服务的客户需求量
                        operation.sumOfCustomerPi = 0
                        operation.ReducedCost = operation.operationTimeCost - operation.sumOfCustomerPi
                        Operation_Dict[f"A_{[]}_{i}_{j}_{t}"] = operation
    print("A类操作构建完毕")

    """---------------第B类操作：经行操作---------------（该操作访问的客户为空集[]）"""
    for i in transNetNode:  # 经行操作起点i可以是：交通运输网络节点
        for j in transNetNode:  # 经行操作的终点j可以是：交通运输网络节点
            for t in network.T:  # 经行操作可选择的时间点
                # 什么样情况才会建立这个操作呢？————对于任意的卡车和无人机的、在t时刻从i点出发前往j节点的合体弧，其终点时刻小于等于max（network.T）时，才会建立否则没意义
                for vehicleArc in ([_by_ijt[(i, j, t)]] if (i, j, t) in _by_ijt else []):  # 卡车弧,只要是构建出来的都是存在的，而且是邻接的（改为按 (i,j,t) 取索引，等价于原来扫全表留下的最后一条）
                    T_from_space = vehicleArc.from_space  # i
                    T_to_space = vehicleArc.to_space  # j
                    T_from_time = vehicleArc.from_time  # t
                    T_to_time = vehicleArc.to_time  # s
                    T_from_state = vehicleArc.from_state  # w
                    T_to_state = vehicleArc.to_state  # w_
                    T_cost = vehicleArc.cost  # 弧成本，其实就是时间
                    T_arcType = vehicleArc.arcType
                    if i == T_from_space and j == T_to_space and t == T_from_time:
                        operation = Operation()
                        operation.startID = i  # 操作起点
                        operation.startTime = t  # 操作开始时间
                        operation.endID = j  # 操作终点
                        operation.endTime = T_to_time  # 操作结束时间
                        operation.VehicleId = None
                        operation.operationName = f"B_{[]}_{i}_{j}_{operation.startTime}_{operation.endTime}"
                        operation.key = f"B_{[]}_{i}_{j}_{t}"
                        operation.same = True  # 肯定是同一个
                        operation.droneTimeCost = T_to_time - T_from_time
                        operation.truckTimeCost = T_to_time - T_from_time
                        operation.operationTimeCost = max(operation.droneTimeCost,operation.truckTimeCost)
                        operation.droneServiceID = []  # 经行操作无客户服务
                        operation.truckServiceID = []  # 经行操作无客户服务
                        operation.ServiceID = []  # 该操作无客户服务
                        operation.ServiceDemand = 0  # 该操作所服务的客户需求量
                        operation.sumOfCustomerPi = 0
                        operation.ReducedCost = operation.operationTimeCost - operation.sumOfCustomerPi
                        Operation_Dict[f"B_{[]}_{i}_{j}_{t}"] = operation
    print("B类操作构建完毕")
    """---------------第C类操作：共同服务操作---------------（该操作访问的客户为道路节点j对应的虚拟客户[j’]）"""
    for i in network.physicalCustomer:  # 共同服务操作起点i可以是：某客户对应的实际交通运输网络节点，j与i是相同的
        j=i
        for t in network.T:  # 共同服务操作可选择的时间点
            # 什么样情况才会建立这个操作呢？————两个合体弧首尾相连，操作开头及结尾是相同的两个空间节点
            for vehicleArc_In in _by_ft.get((i, t), ()):  # 卡车弧,只要是构建出来的都是存在的，而且是邻接的（从 i、t 出发的那几条，顺序与全表一致）
                T_from_space_In = vehicleArc_In.from_space  # i
                T_to_space_In = vehicleArc_In.to_space  # j
                T_from_time_In = vehicleArc_In.from_time  # t
                T_to_time_In = vehicleArc_In.to_time  # s
                T_from_state_In = vehicleArc_In.from_state  # w
                T_to_state_In = vehicleArc_In.to_state  # w_
                T_cost_In = vehicleArc_In.cost  # 弧成本，其实就是时间
                T_arcType_In = vehicleArc_In.arcType
                if i == T_from_space_In and t == T_from_time_In and T_arcType_In == 4 and T_to_space_In !=network.O and T_to_space_In !=network.D:  # 从i节点(非起终点)、t时刻出发的服务弧，对于network.physicalCustomer来说只有客户服务弧了
                    customerID = int(T_to_space_In)  # 所服务的虚拟客户ID（pandas 读进来是 numpy 标量，numpy 2.x 下会把 f"{[customerID]}" 打成 [np.int64(30)]，键就对不上了）
                    for vehicleArc_Out in _by_ft.get((customerID, T_to_time_In), ()):  # 卡车弧,只要是构建出来的都是存在的，而且是邻接的（从虚拟客户点、回到主网的那一刻出发的那几条）
                        T_from_space_Out = vehicleArc_Out.from_space  # i
                        T_to_space_Out = vehicleArc_Out.to_space  # j
                        T_from_time_Out = vehicleArc_Out.from_time  # t
                        T_to_time_Out = vehicleArc_Out.to_time  # s
                        T_from_state_Out = vehicleArc_Out.from_state  # w
                        T_to_state_Out = vehicleArc_Out.to_state  # w_
                        T_cost_Out = vehicleArc_Out.cost  # 弧成本，其实就是时间
                        T_arcType_Out = vehicleArc_Out.arcType

                        if T_from_space_Out == customerID and T_from_time_Out == T_to_time_In and T_arcType_In == 4:
                            operation = Operation()
                            operation.startID = i  # 操作起点
                            operation.startTime = t  # 操作开始时间
                            operation.endID = j  # 操作终点
                            operation.endTime = T_to_time_Out  # 操作结束时间
                            operation.VehicleId = None
                            operation.operationName = f"C_{[customerID]}_{i}_{j}_{operation.startTime}_{operation.endTime}"
                            operation.key = f"C_{[customerID]}_{i}_{j}_{t}"
                            operation.same = True  # 肯定是同一个
                            operation.droneTimeCost = T_to_time_Out - T_from_time_In
                            operation.truckTimeCost = T_to_time_Out - T_from_time_In
                            operation.operationTimeCost = max(operation.droneTimeCost, operation.truckTimeCost)
                            operation.droneServiceID = [customerID]  # 共同服务操作，只服务一个客户
                            operation.truckServiceID = [customerID]  # 共同服务操作，只服务一个客户
                            operation.ServiceID = [customerID]  # 该操作只有一个客户服务
                            operation.ServiceDemand = network.demand[network.Customer.index(customerID)]  # 该操作所服务的客户需求量
                            operation.sumOfCustomerPi = 0  # 调整前默认值
                            operation.ReducedCost = operation.operationTimeCost - operation.sumOfCustomerPi
                            Operation_Dict[f"C_{[customerID]}_{i}_{j}_{t}"] = operation
    print("C类操作构建完毕")
    """---------------第D类操作：无人机服务-卡车经行操作（涉及到排列组合与优化）----【除了发射和回收弧，其他弧的状态都是0】该操作访问的客户为无人机可访问的单个客户"""
    for originS in Candidate_rendezvous:  # 无人机服务-卡车经行操作起点i可以是：潜在起降点
        for destinationS in Candidate_rendezvous:  # 无人机服务-卡车经行操作的终点j可以是：潜在起降点（i可以等于j）
            for originT in network.T[2:-2]:  # 经行操作可选择的出发时间点（从T中的第三个开始，到倒数第三个结束，不算起降）
                # 什么情况才会建立这个操作呢？————（1）可行，续航、容量上可行,在该类操作中因为卡车不访问客户，所以只看无人机；（2）最优，考虑乘子之后，这个操作花费的成本是最小的，由于卡车不访问客户，这个也是看无人机
                """(一)、先找出卡车在t时刻从i出发，前往j节点的时空最短路（dijkstra），然后再将无人机可访问的客户加上去"""
                result = shortest_path_D(Bx, originS, originT, destinationS, T_max=network.T[-3])  # 最短路成本、最短路、network.T[-3]是倒数第三个数
                if result:  # 存在卡车路径，可能存在操作
                    Truck_SPvalue = result[0]  # 卡车最短路的长度（时间衡量）
                    destinationT = originT + Truck_SPvalue  # 卡车到达destinationS的时间

                    """(二)、在时刻{originT}从{originS}前往{destinationS}的卡车最短路已经建立完毕，将无人机可访问的客户加上去【容量、时间窗、续航】"""
                    # 筛选出最优的sortie
                    bestSorties = []  # 保留筛选之后最优的那些架次
                    for customerID in network.Customer:  # 遍历客户节点，找在originS，originT出发，前往服务customerID，并飞到destinationS，用时最小的那个架次
                        sortie = sortie_dict.get((originS, originT, destinationS, customerID), None)
                        bestSorties.append(sortie)  # 可能有None
                    # 与卡车最短路进行拼接
                    for sortie in bestSorties:
                        if sortie is not None:
                            _d_drone = network.demand[network.Customer.index(int(sortie.delivery_space))]  # [release]
                            _limit = UAVendurance - 2 - (_d_drone if CHARGE_PAYLOAD_WHILE_HOVERING else 0)  # [release] original test: Truck_SPvalue <= UAVendurance-2
                            if Truck_SPvalue <= _limit:
                                # 找到起点时间、起点位置、终点位置与当前这个卡车最短路吻合，且无人机的续航可以等待卡车到达该操作终点
                                customerID = int(sortie.delivery_space)  # 同上，转成内置 int 再进键
                                if sortie.end_time <= destinationT:  # 若无人机早到，则等待至卡车抵达(续航是可以坚持到抵达的)，操作终点时间就是卡车的终点时间（两者取max）
                                    operation = Operation()
                                    operation.startID = originS  # 操作起点
                                    operation.startTime = originT-1  # 操作开始时间  这里生成操作的时候，是从发射之后算起的，需要往前推一个发射时间sL
                                    operation.endID = destinationS  # 操作终点
                                    operation.endTime = destinationT+1  # 操作结束时间,同理往后推一个回收时间sR，下同（只有D和E类操作需要这样做）
                                    operation.VehicleId = None
                                    operation.operationName = f"D_{[customerID]}_{originS}_{destinationS}_{operation.startTime}_{operation.endTime}"
                                    operation.key = f"D_{[customerID]}_{originS}_{destinationS}_{originT}"
                                    operation.same = False  # 卡车和无人机路径不相同
                                    operation.droneTimeCost = sortie.end_time - sortie.start_time
                                    operation.truckTimeCost = destinationT - originT
                                    operation.operationTimeCost = max(operation.droneTimeCost, operation.truckTimeCost)+2  # 只有D类和E类操作才需要这样，找回失去的发射回收时间
                                    operation.droneServiceID = [customerID]  # 只服务一个客户
                                    operation.truckServiceID = []  # 卡车经行客户
                                    operation.ServiceID = [customerID]  # 该操作只有一个客户服务
                                    operation.ServiceDemand = network.demand[network.Customer.index(customerID)]  # 该操作所服务的客户需求量
                                    operation.sumOfCustomerPi = 0  # 调整前默认值
                                    operation.ReducedCost = operation.operationTimeCost - operation.sumOfCustomerPi
                                    Operation_Dict[f"D_{[customerID]}_{originS}_{destinationS}_{originT}"] = operation
                                    # print(f"已建立操作D_{[customerID]}_{originS}_{destinationS}_{originT}")
                                elif sortie.end_time > destinationT:  # 若卡车早到，则等待至无人机抵达，操作终点时间就是无人机的终点时间（两者取max）
                                    operation = Operation()
                                    operation.startID = originS  # 操作起点位置
                                    operation.startTime = originT-1  # 操作开始时间
                                    operation.endID = destinationS  # 操作终点位置
                                    operation.endTime = sortie.end_time+1  # 操作结束时间
                                    operation.VehicleId = None
                                    operation.operationName = f"D_{[customerID]}_{originS}_{destinationS}_{operation.startTime}_{operation.endTime}"  # 这个名字跟操作的key在一些时间上有所不同，名字考虑了发射和起降，用于存放在列池，而key没有考虑发射和起降，仅仅是继承自构建的规则而设计的
                                    operation.key = f"D_{[customerID]}_{originS}_{destinationS}_{originT}"
                                    operation.same = False  # 卡车和无人机路径不相同
                                    operation.droneTimeCost = sortie.end_time - sortie.start_time
                                    operation.truckTimeCost = destinationT - originT
                                    operation.operationTimeCost = max(operation.droneTimeCost, operation.truckTimeCost)+2
                                    operation.droneServiceID = [customerID]  # 只服务一个客户
                                    operation.truckServiceID = []  # 卡车经行客户
                                    operation.ServiceID = [customerID]  # 该操作只有一个客户服务
                                    operation.ServiceDemand = network.demand[network.Customer.index(customerID)]  # 该操作所服务的客户需求量
                                    operation.sumOfCustomerPi = 0  # 调整前默认值
                                    operation.ReducedCost = operation.operationTimeCost - operation.sumOfCustomerPi
                                    Operation_Dict[f"D_{[customerID]}_{originS}_{destinationS}_{originT}"] = operation
                                    # print(f"已建立操作D_{[customerID]}_{originS}_{destinationS}_{originT}")
    print("D类操作构建完毕")
    """---------------第E类操作：无人机服务-卡车服务+经行操作（涉及到排列组合与优化）-------（要用极小的乘子值来指导最短路生成，同时也要用禁忌表）"""
    # 初始化路径缓存。
    # 组合是按规模 r 从小到大成段生成的，同一段里卡车客户集合的大小恒为 r-1，
    # 而跨段唯一还有用的信息是「某个集合从某个 (起点, 出发时刻) 根本服务不完」——
    # 子集服务不完，超集必然也服务不完。所以：
    #   path_cache 只存本段、且确实走得通的结果，进入下一段即清空；
    #   走不通的记录压成每个集合一个整数位图（位号 = 起点序号*时间跨度 + 出发时刻），
    #   只把上一段的位图留给本段做剪枝。
    # 这样占用与「走得通的组合数」同阶，不再随枚举量线性膨胀。
    path_cache = {}
    infeas_cur = {}
    infeas_prev = {}
    keyPool = {}                      # 集合元组只保留一份，省掉重复的元组对象
    _crIdx = {_c: _k for _k, _c in enumerate(Candidate_rendezvous)}
    _TCODE = int(network.T[-1]) + 1   # network.T 是 numpy 数组，取出来还是 numpy 标量
    _curTruckSize = -1
    cache_hits = 0
    cache_misses = 0
    # 预处理运输节点间的最短时间（Dijkstra）
    dist_matrix = precompute_dist_matrix(Bx, Candidate_rendezvous, network.T[-3])
    # 生成客户组合
    customerSubset = generate_combinations_lazy(network.Customer, network.demand, Truckcapacity, UAVcapacity)  # 惰性枚举，次序与 generate_combinations 一致
    # 只有存在架次的 (起点, 终点, 出发时刻) 才可能建成 E 类操作。原先 12×12×60 的三重
    # 循环里绝大多数组合都是 bestSortie 取回 None、什么也不做的空转，客户数一大全耗在这。
    # 这里把 sortie_dict 按无人机所服务的客户倒排一次，循环只走真正有架次的组合；
    # 键的构成、成本比较与覆盖逻辑一概不动。
    _CR = set(Candidate_rendezvous)
    _crIdx0 = {_c: _k for _k, _c in enumerate(Candidate_rendezvous)}
    # 架次按「无人机客户 -> (起点, 出发时刻) -> 各终点」分组。
    # 卡车能否从 (起点, 出发时刻) 出发服务完 truckSet 与终点无关，所以走不通位图、缓存向量
    # 都只需按组查一次；原来按 (起点, 终点, 出发时刻) 逐条查，绝大多数是同一组里对
    # 同一个「走不通」结论的重复查询（SF10 上占命中的 97%）。
    # 一个分工方案内每个 key 只会被碰到一次（sortie_dict 的键唯一），所以先把本方案
    # 产生的操作收进缓冲区，再按原三重循环的次序 (起点序, 终点序, 出发时刻) 写入
    # Operation_Dict，写入先后与比较覆盖的语义都与原实现一致，文件行序也一样。
    _groups_tmp = {}
    for (_oS, _oT, _dS, _cid), _s in sortie_dict.items():
        if _oS in _CR and _dS in _CR and _oT in _Tmid:
            _groups_tmp.setdefault(_cid, {}).setdefault((_oS, int(_oT)), []).append((_dS, _s))
    sortie_groups = {}
    for _cid, _g in _groups_tmp.items():
        _lst = []
        for (_oS, _oT), _ds in _g.items():
            _ds.sort(key=lambda _e: _crIdx0[_e[0]])
            _dests = tuple((_dS, dist_matrix[_oS][_dS], _s) for _dS, _s in _ds
                           if dist_matrix[_oS][_dS] != math.inf)
            if _dests:
                _lst.append((_oS, _oT, _crIdx0[_oS] * _TCODE + _oT, _dests))
        _lst.sort(key=lambda _e: (_crIdx0[_e[0]], _e[1]))
        sortie_groups[_cid] = tuple(_lst)
    del _groups_tmp
    _Tend = int(network.T[-3])
    _sS2 = 2 * network.sS
    _endur = UAVendurance - 2
    import time as _tm
    _t0 = _tm.perf_counter()
    _spe_t = [0.0]
    for i, subset in enumerate(customerSubset):  # subset：同一客户集合的各种分工方案，如 [(['c2', 'c4'], 'c1'), (['c1', 'c2'], 'c4')]
        if i % 20 == 0:
            print("E进度 %d 用时%.1fs 未命中%d 命中%d SPE用时%.1fs 卡车集大小%d 操作数%d"
                  % (i, _tm.perf_counter() - _t0, cache_misses,
                     cache_hits, _spe_t[0], len(subset[0][0]), len(Operation_Dict)), flush=True)
            print("      缓存 集合数%d 条目数%d 走不通集合%d 池%d 当前层%d"
                  % (len(path_cache), sum(len(_v) for _v in path_cache.values()),
                     len(infeas_cur), len(keyPool), _curTruckSize), flush=True)
        if len(subset[0][0]) != _curTruckSize:   # 进入下一段（卡车客户集合大了一个）
            if _curTruckSize >= 1 and not path_cache:
                # 上一段没有任何「(卡车客户集合, 起点, 出发时刻) 在续航界内走得通」的记录。
                # 超集在任一节点的最早完成时刻不早于子集，所以更大的集合从任何 (起点, 出发时刻)
                # 出发都服务不完，后面各段不可能再产生 E 类操作，直接结束枚举，结果不变。
                print("E类枚举提前结束：卡车客户集合大小 %d 这一段无可行结果，更大的集合必然不可行（已枚举 %d 个子集）"
                      % (_curTruckSize, i), flush=True)
                break
            _curTruckSize = len(subset[0][0])
            infeas_prev = infeas_cur
            infeas_cur = {}
            path_cache = {}
            keyPool = {}
        for j in range(0, len(subset)):  # 对应的不同组合安排，如：(['c2', 'c4'], 'c1')，卡车访问c2、c4，无人机访问c1
            assignment = subset[j]  # 如：(['c2', 'c4'], 'c1')
            truckSet = assignment[0]  # 记录卡车所服务的客户集合，在建立操作时候要用
            droneID = assignment[1]
            customerID_list = sorted(truckSet + [droneID])

            demandSum = 0  # 所服务的客户的需求量求和
            for _j in customerID_list:
                demandSum += network.demand[network.Customer.index(_j)]

            _truckKey = tuple(sorted(truckSet))
            _truckKey = keyPool.setdefault(_truckKey, _truckKey)
            _truckExtra = len(truckSet) * _sS2
            _keyHead = f"E_{customerID_list}_"
            _buf = []
            for originS, originT, _code, _dests in sortie_groups.get(droneID, ()):
                # 原实现先按 (起点, 终点) 的最短时间做一次时域过滤，全组都过不了就不会去搜；这里保持一致
                _live = [_d for _d in _dests if originT + _d[1] + _truckExtra <= _Tend]
                if not _live:
                    continue
                _bad = infeas_cur.get(_truckKey, 0)
                if (_bad >> _code) & 1:                    # 本段已判定走不通
                    cache_hits += len(_live)
                    continue
                _slot = path_cache.get(_truckKey)
                _vec = _slot.get(_code) if _slot is not None else None
                if _vec is not None:
                    cache_hits += len(_live)
                else:
                    cache_hits += len(_live) - 1
                    _pruned = False
                    if _curTruckSize >= 3:
                        for _c in _truckKey:
                            if (infeas_prev.get(tuple(_x for _x in _truckKey if _x != _c), 0)
                                    >> _code) & 1:
                                _pruned = True
                                break
                    if _pruned:
                        infeas_cur[_truckKey] = _bad | (1 << _code)
                        continue
                    _ts = _tm.perf_counter()
                    # 搜索只需推进到 originT + (续航 - 2)：建立 E 类操作要求 Truck_SPvalue <= UAVendurance - 2，
                    # 更晚的结果用不上；「界内服务不完」对超集同样成立，跨段剪枝仍然正确。
                    _best = shortest_path_E_multi(Bx, originS, originT, truckSet,
                                                  T_max=min(_Tend, originT + _endur))
                    _spe_t[0] += _tm.perf_counter() - _ts
                    cache_misses += 1
                    if _best:
                        _vec = bytes(_best.get(_r, 255) for _r in Candidate_rendezvous)
                        if _slot is None:
                            _slot = path_cache[_truckKey] = {}
                        _slot[_code] = _vec
                    else:
                        infeas_cur[_truckKey] = _bad | (1 << _code)
                        continue
                for destinationS, _min_time, bestSortie in _live:
                    _tEnd = _vec[_crIdx[destinationS]]
                    if _tEnd == 255:
                        continue
                    Truck_SPvalue = _tEnd - originT              # 卡车最短路用时
                    if not Truck_SPvalue:
                        continue
                    destinationT = originT + Truck_SPvalue       # 卡车到达 destinationS 的时间
                    _d_drone = network.demand[network.Customer.index(int(droneID))]  # [release]
                    _limit = _endur - (_d_drone if CHARGE_PAYLOAD_WHILE_HOVERING else 0)  # [release] original test: Truck_SPvalue <= _endur
                    if Truck_SPvalue <= _limit:
                        operation = Operation()
                        operation.startID = originS
                        operation.startTime = originT - 1
                        operation.endID = destinationS
                        # 无人机早到则等卡车，卡车早到则等无人机：终点时刻取两者较大者
                        operation.endTime = (destinationT if bestSortie.end_time <= destinationT else bestSortie.end_time) + 1
                        operation.VehicleId = None
                        operation.operationName = f"E_{customerID_list}_{originS}_{destinationS}_{operation.startTime}_{operation.endTime}"
                        operation.key = f"E_{customerID_list}_{originS}_{destinationS}_{originT}"
                        operation.same = False
                        operation.droneTimeCost = bestSortie.end_time - bestSortie.start_time
                        operation.truckTimeCost = destinationT - originT
                        operation.operationTimeCost = max(operation.droneTimeCost, operation.truckTimeCost) + 2
                        operation.droneServiceID = [droneID]
                        operation.truckServiceID = truckSet
                        operation.ServiceID = customerID_list
                        operation.ServiceDemand = demandSum
                        operation.sumOfCustomerPi = 0
                        operation.ReducedCost = operation.operationTimeCost - operation.sumOfCustomerPi
                        _buf.append((_crIdx[originS], _crIdx[destinationS], originT, operation))
            if _buf:
                _buf.sort(key=lambda _e: (_e[0], _e[1], _e[2]))
                for _, _, _, operation in _buf:
                    _k = operation.key
                    _old = Operation_Dict.get(_k)
                    if _old is not None:
                        # 与原实现一致：拿不含起降的 max(无人机用时, 卡车用时) 去比已存操作的 operationTimeCost（含 +2）
                        if operation.operationTimeCost - 2 < _old.operationTimeCost:
                            Operation_Dict[_k] = operation
                    else:
                        Operation_Dict[_k] = operation
    print(f"缓存命中率: {cache_hits / (cache_hits + cache_misses+0.00000000000001):.1%}")
    import os as _os
    _cache_dir = _os.environ.get('CACHE_RATE_DIR')
    if _cache_dir:
        result_Path = _os.path.join(_cache_dir, f'cache_rate_{len(network.Customer)}.txt')
        with open(result_Path, 'w') as _f:
            _f.write(f'{cache_hits / (cache_hits + cache_misses+0.00000000000001):.1%}')
    print(f"操作构建完毕，总数量是：{len(Operation_Dict)}")
    return Operation_Dict


if __name__ == "__main__":
    import constructStsNetwork as construct  # [release] moved here from the top of the file; only the self-test below uses it
    # 网络的基本参数
    CustomerLst = [25, 26, 27, 28]  # 虚拟客户点id
    physicalCustomerList = [4, 12, 14, 18]  # 实际的客户点id
    demand = [3, 20, 9, 4]  # 客户需求量
    O = 0
    D = 29
    UAVendurance = 10
    UAVcapacity = 5
    Truckcapacity = 20  # 卡车容量
    Vehicle_Cap = [25, 25]  # 卡车组容量
    vehicleNum = 2  # 车辆数量
    vehicleLst = [i for i in range(0, vehicleNum)]
    sL = 1
    sR = 1
    sS = 1

    # 每次迭代中的客户乘子
    pi_DICT = {'25': 34, '26': 41.0, '27': 0.0, '28': 0.0}

    # 网络的构建
    st_network = construct.network
    st_network.readStsNode(physicalCustomerList, CustomerLst, vehicleLst, demand, Vehicle_Cap,Truckcapacity,UAVcapacity, UAVendurance, sL, sR, sS, O, D)
    st_network.readStsVehicleArc()
    st_network.readPotentialSorties(sL, sR)
    st_network.readStsDroneArc()

    # 操作的构建（构建+调整乘子）
    # 1.构建Operations
    Operations = FEoperationFunction(st_network,Truckcapacity,UAVcapacity,UAVendurance)
    # 2.调整op.ReducedCost
    for op in Operations.values():
        piSum = 0
        for custerName in op.ServiceID:
            piSum += pi_DICT[f'{custerName}']
        op.sumOfCustomerPi = piSum
        op.ReducedCost = op.operationTimeCost - op.sumOfCustomerPi

    # print(f"操作总数量是：{len(Operations)}")
    for op in Operations.values():
        print(op.operationName,"结束时间：",op.endTime, "RC:",op.ReducedCost,"乘子：", op.sumOfCustomerPi,"卡车用时：",op.truckTimeCost,"无人机用时：",op.droneTimeCost,"总操作用时(DE类含起降)：",op.operationTimeCost)
