# -*- coding: utf-8 -*-
# Construction of the feasible and efficient space-time operations (types A to E) used by the pricing dynamic program; reference implementation imported by the B&P drivers.
# Original file name in the authors' project: constructOperation.py (last edited 2024/12/12)
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
    # 构建时空弧的邻接表
    adjacency = {}
    for i, j, t, s in Bx:
        if (i, t) not in adjacency:
            adjacency[(i, t)] = []
        adjacency[(i, t)].append((j, s))
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

    # 构建时空邻接表（预处理提高效率）
    adjacency = defaultdict(list)
    for i, j, t, s in Bx:
        adjacency[(i, t)].append((j, s, (s - t)))  # 存储节点、到达时间、时间消耗

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
                idx = customerSubsets.index(v)
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
    matrix = np.inf * np.ones((size, size))  # 默认标签是较大值
    for i in range(0, size):
        matrix[i][i] = np.inf  # 不存在瞬间移动
    for vehicleArc in network.stsVehicleArcList:  # 卡车的时空状态弧，这里只用w=w’=1的时空状态弧
        if vehicleArc.from_state == 1 and vehicleArc.to_state == 1 and vehicleArc.from_space in transNetNode and vehicleArc.to_space in transNetNode and vehicleArc.from_time in network.T[2:-2] and vehicleArc.to_time in network.T[2:-2]:  # 分离之后的卡车弧
            fromSpace = vehicleArc.from_space
            fromTime = vehicleArc.from_time
            toSpace = vehicleArc.to_space
            toTime = vehicleArc.to_time
            rowIndex = STtransNetNode.index((fromSpace, fromTime))  # 这个时空起点对应距离矩阵的第几行
            colIndex = STtransNetNode.index((toSpace, toTime))  # 这个时空起点对应距离矩阵的第几列
            matrix[rowIndex][colIndex] = toTime - fromTime  # 这里邻接矩阵成本就是弧的时间
    # print(matrix)
    # print(matrix[0])
    Operation_Dict = {}
    """---------------第A类操作：出车&入库操作---------------（该操作访问的客户为空集[]）"""
    # (1-1) 出车操作
    for i in [network.O]:  # 出车操作起点i可以是：起点车场O
        for j in [network.O, network.D, full_list[1]]: # 出车操作的终点j可以是：起点车场O、终点车场D、第一个道路节点（索引一般就是1）
            for t in network.T:  # 出车操作可选择的时间点
                # 什么样情况才会建立这个操作呢？————对于任意的卡车和无人机的、在t时刻从i点出发前往j节点的合体弧，其终点时刻小于等于max（network.T）时，才会建立否则没意义
                for vehicleArc in network.stsVehicleArcList:  # 卡车弧,只要是构建出来的都是存在的，而且是邻接的
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
                for vehicleArc in network.stsVehicleArcList:  # 卡车弧,只要是构建出来的都是存在的，而且是邻接的
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
                for vehicleArc in network.stsVehicleArcList:  # 卡车弧,只要是构建出来的都是存在的，而且是邻接的
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
            for vehicleArc_In in network.stsVehicleArcList:  # 卡车弧,只要是构建出来的都是存在的，而且是邻接的
                T_from_space_In = vehicleArc_In.from_space  # i
                T_to_space_In = vehicleArc_In.to_space  # j
                T_from_time_In = vehicleArc_In.from_time  # t
                T_to_time_In = vehicleArc_In.to_time  # s
                T_from_state_In = vehicleArc_In.from_state  # w
                T_to_state_In = vehicleArc_In.to_state  # w_
                T_cost_In = vehicleArc_In.cost  # 弧成本，其实就是时间
                T_arcType_In = vehicleArc_In.arcType
                if i == T_from_space_In and t == T_from_time_In and T_arcType_In == 4 and T_to_space_In !=network.O and T_to_space_In !=network.D:  # 从i节点(非起终点)、t时刻出发的服务弧，对于network.physicalCustomer来说只有客户服务弧了
                    customerID = T_to_space_In  # 所服务的虚拟客户ID
                    for vehicleArc_Out in network.stsVehicleArcList:  # 卡车弧,只要是构建出来的都是存在的，而且是邻接的
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
                                customerID = sortie.delivery_space
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
    # 初始化路径缓存
    path_cache = {}
    cache_hits = 0
    cache_misses = 0
    # 预处理运输节点间的最短时间（Dijkstra）
    dist_matrix = precompute_dist_matrix(Bx, Candidate_rendezvous, network.T[-3])
    # 生成客户组合
    customerSubset = generate_combinations(network.Customer, network.demand, Truckcapacity, UAVcapacity)  # 建立卡车可以访问的客户集合 customerSubset =[[],[],[],[],...[]],集合中每个元素一定存在至少一个符合容量约束的客户（求和小于卡车容量）
    # print("卡车可以访问的客户集合customerSubset:",customerSubset)
    for i in range(0, len(customerSubset)):
        print(f"E类操作构建进度：{i / len(customerSubset)}")
        subset = customerSubset[i]  # 这个操作所服务的对应的所有客户组成，例如都是c1、c2、c4：[(['c2', 'c4'], 'c1'), (['c1', 'c2'], 'c4')]
        for j in range(0, len(subset)):  # 对应的不同组合安排，如：(['c2', 'c4'], 'c1')，卡车访问c2、c4，无人机访问c1
            assignment = subset[j]  # 如：(['c2', 'c4'], 'c1')
            truckSet = assignment[0]  # 记录卡车所服务的客户集合，在建立操作时候要用
            # print("truckSet:",truckSet)
            droneID = assignment[1]
            customerID_list = sorted(truckSet + [droneID])

            demandSum = 0  # 所服务的客户的需求量求和
            for _j in customerID_list:
                demandSum += network.demand[network.Customer.index(_j)]

            for originS in Candidate_rendezvous:  # 无人机服务-卡车服务+经行操作起点i可以是：交通运输网络节点
                for destinationS in Candidate_rendezvous:  # 无人机服务-卡车服务+经行操作的终点j可以是：交通运输网络节点（i可以等于j）
                    min_time = dist_matrix[originS][destinationS] # 获取最短时间（不访问客户）
                    if min_time == math.inf:
                        continue
                    for originT in network.T[2:-2]:  # 操作可选择的出发时间点（从T中的第三个开始，到倒数第三个结束，不算起降）
                        # 什么情况才会建立这个操作呢？————
                        """(一)、先找出卡车在originT时刻从originS出发，前往destinationS节点，服务客户集合truckSet内客户的时空最短路，然后再将无人机可访问的客户droneID加上去"""
                        # 计算卡车路径的最短时间（使用预计算矩阵）（不访问客户）
                        truck_time = min_time + len(truckSet) * 2 * network.sS
                        if originT + truck_time > network.T[-3]:
                            continue
                        # ========== 新增缓存检查 ==========
                        # print("缓存检查")
                        # 生成唯一缓存键：包含起终点、时间、客户集合
                        cache_key = (originS, destinationS, originT, tuple(sorted(truckSet)))
                        # 检查缓存
                        if cache_key in path_cache:
                            result = path_cache[cache_key]
                            cache_hits += 1
                        else:
                            result = shortest_path_E(Bx, originS, originT, destinationS, truckSet, T_max = network.T[-3])  # 最短路成本、最短路
                            path_cache[cache_key] = result
                            cache_misses += 1
                        # print("result",result)
                        if result:  # 存在卡车路径，可能存在操作
                            Truck_SPvalue = result  # 卡车最短路的长度（时间衡量）
                            destinationT = originT + Truck_SPvalue  # 卡车到达destinationS的时间
                            """(二)、在时刻{originT}从{originS}前往{destinationS}的卡车最短路已经建立完毕，将无人机访问的客户droneID加上去【容量、时间窗、续航】"""
                            # 筛选出最优的sortie（且一定是卡车服务了）
                            # print("sortie_dict.keys:",sortie_dict.keys())

                            bestSortie = sortie_dict.get((originS, originT, destinationS, droneID), None)

                            if bestSortie is not None:
                                # print(f"卡车在时刻{originT}从{originS}出发、前往{destinationS},并服务客户{truckSet},的最短路长度为{Truck_SPvalue}")
                                # print(f"无人机在时刻{originT}从{originS}出发、前往{destinationS},服务客户{droneID}的架次是{bestSortie.PotentialSorties}，架次用时（含起降）：{bestSortie.end_time-bestSortie.start_time+2}”")
                                _d_drone = network.demand[network.Customer.index(int(droneID))]  # [release]
                                _limit = UAVendurance - 2 - (_d_drone if CHARGE_PAYLOAD_WHILE_HOVERING else 0)  # [release] original test: Truck_SPvalue <= UAVendurance - 2
                                if Truck_SPvalue <= _limit:
                                    # print("___________此时用时符合要求（无坠机风险）___________")
                                    # 找到起点时间、起点位置、终点位置与当前这个卡车最短路吻合，且无人机的续航可以等待卡车到达该操作终点
                                    if bestSortie.end_time <= destinationT:  # 若无人机早到，则等待至卡车抵达(续航是可以坚持到抵达的)，操作终点时间就是卡车的终点时间（两者取max）
                                        # 先判断之前建立的操作中，是否已经有E类的相同客户集合的了，有的话比较，胜者更新；没有的话直接新建并存储
                                        if f"E_{customerID_list}_{originS}_{destinationS}_{originT}" in Operation_Dict:
                                            # print("已经存在于操作字典中")
                                            formerCost = Operation_Dict[f"E_{customerID_list}_{originS}_{destinationS}_{originT}"].operationTimeCost
                                            currentCost = max(bestSortie.end_time - bestSortie.start_time, destinationT - originT)
                                            if currentCost < formerCost: # 当前经过customerID_list的操作成本，小于之前经过customerID_list操作的成本，则将当前操作覆盖
                                                operation = Operation()
                                                operation.startID = originS  # 操作起点
                                                operation.startTime = originT-1  # 操作开始时间
                                                operation.endID = destinationS  # 操作终点
                                                operation.endTime = destinationT+1  # 操作结束时间
                                                operation.VehicleId = None
                                                operation.operationName = f"E_{customerID_list}_{originS}_{destinationS}_{operation.startTime}_{operation.endTime}"
                                                operation.key = f"E_{customerID_list}_{originS}_{destinationS}_{originT}"
                                                operation.same = False  # 卡车和无人机路径不相同
                                                operation.droneTimeCost = bestSortie.end_time - bestSortie.start_time
                                                operation.truckTimeCost = destinationT - originT
                                                operation.operationTimeCost = max(operation.droneTimeCost,operation.truckTimeCost)+2
                                                operation.droneServiceID = [droneID]  # 无人机访问的客户
                                                operation.truckServiceID = truckSet  # 卡车访问的客户
                                                operation.ServiceID = customerID_list  # 该操作所有服务的客户
                                                operation.ServiceDemand = demandSum
                                                operation.sumOfCustomerPi = 0  # 调整前默认值
                                                operation.ReducedCost = operation.operationTimeCost - operation.sumOfCustomerPi
                                                Operation_Dict[f"E_{customerID_list}_{originS}_{destinationS}_{originT}"] = operation
                                                # print(f"已建立操作E_{customerID_list}_{originS}_{destinationS}_{originT}")
                                            else:
                                                pass
                                        if f"E_{customerID_list}_{originS}_{destinationS}_{originT}" not in Operation_Dict:
                                            # print("不存在于操作字典中")
                                            operation = Operation()
                                            operation.startID = originS  # 操作起点
                                            operation.startTime = originT-1  # 操作开始时间
                                            operation.endID = destinationS  # 操作终点
                                            operation.endTime = destinationT+1  # 操作结束时间
                                            operation.VehicleId = None
                                            operation.operationName = f"E_{customerID_list}_{originS}_{destinationS}_{operation.startTime}_{operation.endTime}"
                                            operation.key = f"E_{customerID_list}_{originS}_{destinationS}_{originT}"
                                            operation.same = False  # 卡车和无人机路径不相同
                                            operation.droneTimeCost = bestSortie.end_time - bestSortie.start_time
                                            operation.truckTimeCost = destinationT - originT
                                            operation.operationTimeCost = max(operation.droneTimeCost,operation.truckTimeCost)+2
                                            operation.droneServiceID = [droneID]  # 无人机访问的客户
                                            operation.truckServiceID = truckSet # 卡车访问的客户
                                            operation.ServiceID = customerID_list  # 该操作所有服务的客户
                                            operation.ServiceDemand = demandSum
                                            operation.sumOfCustomerPi = 0  # 调整前默认值
                                            operation.ReducedCost = operation.operationTimeCost - operation.sumOfCustomerPi
                                            Operation_Dict[f"E_{customerID_list}_{originS}_{destinationS}_{originT}"] = operation
                                            # print(f"已建立操作E_{customerID_list}_{originS}_{destinationS}_{originT}")

                                    if bestSortie.end_time > destinationT:  # 若卡车早到，则等待至无人机抵达，操作终点时间就是无人机的终点时间（两者取max）
                                        # 先判断之前建立的操作中，是否已经有E类的相同客户集合的了，有的话比较，胜者更新；没有的话直接新建并存储
                                        if f"E_{customerID_list}_{originS}_{destinationS}_{originT}" in Operation_Dict:
                                            # print("已经存在于字典中")
                                            formerCost = Operation_Dict[f"E_{customerID_list}_{originS}_{destinationS}_{originT}"].operationTimeCost
                                            currentCost = max(bestSortie.end_time - bestSortie.start_time,destinationT - originT)
                                            if currentCost < formerCost:  # 当前经过customerID_list的操作成本，小于之前经过customerID_list操作的成本，则将当前操作覆盖
                                                operation = Operation()
                                                operation.startID = originS  # 操作起点位置
                                                operation.startTime = originT-1  # 操作开始时间
                                                operation.endID = destinationS  # 操作终点位置
                                                operation.endTime = bestSortie.end_time+1  # 操作结束时间
                                                operation.VehicleId = None
                                                operation.operationName = f"E_{customerID_list}_{originS}_{destinationS}_{operation.startTime}_{operation.endTime}"
                                                operation.key = f"E_{customerID_list}_{originS}_{destinationS}_{originT}"
                                                operation.same = False  # 卡车和无人机路径不相同
                                                operation.droneTimeCost = bestSortie.end_time - bestSortie.start_time
                                                operation.truckTimeCost = destinationT - originT
                                                operation.operationTimeCost = max(operation.droneTimeCost,operation.truckTimeCost)+2
                                                operation.droneServiceID = [droneID]  # 无人机服务的客户
                                                operation.truckServiceID = truckSet  # 卡车服务的客户
                                                operation.ServiceID = customerID_list  # 该操作所有服务的客户
                                                operation.ServiceDemand = demandSum
                                                operation.sumOfCustomerPi = 0  # 调整前默认值
                                                operation.ReducedCost = operation.operationTimeCost - operation.sumOfCustomerPi
                                                Operation_Dict[f"E_{customerID_list}_{originS}_{destinationS}_{originT}"] = operation
                                                # print(f"已建立操作E_{customerID_list}_{originS}_{destinationS}_{originT}")
                                            else:
                                                pass
                                        if f"E_{customerID_list}_{originS}_{destinationS}_{originT}" not in Operation_Dict:
                                            # print("不存在于操作字典中")
                                            operation = Operation()
                                            operation.startID = originS  # 操作起点位置
                                            operation.startTime = originT-1  # 操作开始时间
                                            operation.endID = destinationS  # 操作终点位置
                                            operation.endTime = bestSortie.end_time+1  # 操作结束时间
                                            operation.VehicleId = None
                                            operation.operationName = f"E_{customerID_list}_{originS}_{destinationS}_{operation.startTime}_{operation.endTime}"
                                            operation.key = f"E_{customerID_list}_{originS}_{destinationS}_{originT}"
                                            operation.same = False  # 卡车和无人机路径不相同
                                            operation.droneTimeCost = bestSortie.end_time - bestSortie.start_time
                                            operation.truckTimeCost = destinationT - originT
                                            operation.operationTimeCost = max(operation.droneTimeCost,operation.truckTimeCost)+2
                                            operation.droneServiceID = [droneID]  # 无人机服务的客户
                                            operation.truckServiceID = truckSet   # 卡车服务的客户
                                            operation.ServiceID = customerID_list  # 该操作所有服务的客户
                                            operation.ServiceDemand = demandSum
                                            operation.sumOfCustomerPi = 0  # 调整前默认值
                                            operation.ReducedCost = operation.operationTimeCost - operation.sumOfCustomerPi
                                            Operation_Dict[f"E_{customerID_list}_{originS}_{destinationS}_{originT}"] = operation
                                            # print(f"已建立操作E_{customerID_list}_{originS}_{destinationS}_{originT}")

                            # print(f"卡车在时刻{originT}从{originS}前往{destinationS},并访问客户{truckSet},的最短路长度为{result[0]}，具体路径为："," -> ".join([f"({node},{time})" for node, time in result[1]]))
                        else:  # 不存在卡车路径，不可能存在操作
                            # print("E类操作居然不存在卡车路径，有点离谱，需要检查函数")
                            pass
                            # print(f"在时刻{originT}从{originS}前往{destinationS},并访问客户{truckSet},无法找到满足约束条件的路径。")
    print(f"缓存命中率: {cache_hits / (cache_hits + cache_misses+0.00000000000001):.1%}")
    result_Path = fr'{RESULT_DIR}/Performance of acceleration components/cache_rate_{len(network.Customer)}.txt'
    result_File = open(result_Path, 'w').close()
    result_generate = open(result_Path, mode='a')
    result_generate.write(str(f'{cache_hits / (cache_hits + cache_misses+0.00000000000001):.1%}') )
    result_generate.close()
    print(f"操作构建完毕，总数量是：{len(Operation_Dict)}")
    return Operation_Dict


if __name__ == "__main__":
    import constructStsNetwork as construct  # [release] moved here from the top of the file; only the self-test below uses it
    # 网络的基本参数
    CustomerLst = [25, 26, 27, 28, 29,30,31,32,33,34,35,36,37]  # 虚拟客户点id
    physicalCustomerList = [4,12,14,18,2,13,6,16,11,10,15,19,20]  # 实际的客户点id
    demand = [4,   7,   3,   5,   6,   3,   6,   4,   7,   5,   6,   3,   5]  # 客户需求量
    O = 0
    D = 38
    UAVendurance = 15
    UAVcapacity = 5
    Truckcapacity = 50  # 卡车容量
    Vehicle_Cap = [50, 50]  # 卡车组容量
    vehicleNum = 3  # 车辆数量
    vehicleLst = [i for i in range(0, vehicleNum)]
    sL = 1
    sR = 1
    sS = 1

    # 每次迭代中的客户约束对偶变量
    # 当前仅用于测试 operation construction，因此初始化为 0
    pi_DICT = {str(customer): 0.0 for customer in CustomerLst}

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
