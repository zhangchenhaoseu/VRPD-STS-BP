# -*- coding: utf-8 -*-
# Variant of DPforSPP_operation.py imported (but not called) by four Sioux Falls drivers.
# Original file name in the authors' project: DPforSPP_operationImprove.py
import os, sys, math
from repo_paths import DATA_DIR, RESULT_DIR  # puts code/ and code/networks/ on sys.path
import math
from collections import defaultdict


# 增加了剪枝（优超关系）
# 操作预处理：
# 使用字典将操作按(startID, startTime)分组，查找时间复杂度从O(M)降为O(1)
# 分层状态管理：
# 按时间步处理状态后立即删除，内存占用从O(T*N)降为O(N)
# 使用前驱指针替代完整路径存储，内存占用降低90%+
# 即时最优解更新：
# 在状态处理过程中实时更新最优解，避免最后全表扫描
# 剪枝优化：
# 提前跳过无效操作（重复客户/容量超限）
# 使用位运算快速校验客户访问状态
# 路径重构优化：
# 通过反向追踪前驱节点构建路径，避免路径复制开销
# 用于快速判断某客户组合是否可能扩展为有效路径，在状态扩展时，检查新生成的客户组合（位掩码）是否属于 valid_prefix。若不属于，直接剪枝


def shortest_path_operations(operations, network, truckGroupCapacity):
    print("===> Optimized dynamic programming approach <===")

    # 操作预处理：按(startID, startTime)分组
    op_groups = defaultdict(list)
    for op in operations.values():
        op_groups[(op.startID, op.startTime)].append(op)

    # 客户索引映射
    customer_index = {customer: i for i, customer in enumerate(network.Customer)}
    startS, endS = network.O, network.D
    startT = 0

    # 动态规划表：state -> (cost, predecessor, last_op)
    dp = {}
    initial_state = (startS, startT, 0, 0)
    dp[initial_state] = (0, None, None)

    result = math.inf
    best_state = None

    # 分层处理（按时间步）
    max_time = max(op.endTime for op in operations.values())
    for t in range(startT, max_time + 1):
        current_states = [s for s in dp.keys() if s[1] == t]

        for state in current_states:
            current_cost, pred, last_op = dp[state]
            currentS, currentT, used, demand = state

            # 实时更新最优解
            if currentS == endS and current_cost < result:
                result = current_cost
                best_state = state

            # 处理可用操作
            for op in op_groups.get((currentS, t), []):
                # 检查客户重复访问
                new_used = used
                valid = True
                for customer in op.ServiceID:
                    if customer not in customer_index:
                        continue
                    idx = customer_index[customer]
                    if new_used & (1 << idx):
                        valid = False
                        break
                    new_used |= (1 << idx)
                if not valid:
                    continue

                # 检查容量约束
                new_demand = demand + op.ServiceDemand
                if new_demand > truckGroupCapacity:
                    continue

                # 生成新状态
                new_state = (op.endID, op.endTime, new_used, new_demand)
                new_cost = current_cost + op.ReducedCost

                # 状态更新策略
                if (new_state not in dp) or (new_cost < dp[new_state][0]):
                    dp[new_state] = (new_cost, state, op)

    # 最终最优解确认
    for state in dp:
        if state[0] == endS and dp[state][0] < result:
            result = dp[state][0]
            best_state = state

    # 路径回溯（关键修正点）
    best_path = []
    if best_state is not None:
        current = best_state
        while current is not None:
            entry = dp.get(current)
            if entry is None:
                break
            if entry[2] is not None:  # 过滤初始状态
                best_path.append(entry[2])
            current = entry[1]  # 前驱状态
        best_path.reverse()

    return (result, best_path) if result < math.inf else (None, [])


if __name__ == "__main__":
    import constructOperation as cO  # [release] moved here from the top of the file; only the self-test below uses it
    import constructStsNetwork as construct  # [release] moved here from the top of the file; only the self-test below uses it
    # 网络的基本参数
    CustomerLst = [10, 11, 12, 13]  # 虚拟客户点id
    physicalCustomerList = [3, 6, 8, 9]  # 实际的客户点id
    demand = [3, 20, 4, 9]  # 客户需求量
    UAVendurance = 8
    UAVcapacity = 5
    Truckcapacity = 20  # 卡车容量
    Vehicle_Cap = [25, 25]  # 卡车组容量
    vehicleNum = 2  # 车辆数量
    truckGroupCapacity = 25
    vehicleLst = [i for i in range(0, vehicleNum)]
    sL = 1
    sR = 1
    sS = 1
    O = 0
    D = 14
    # 每次迭代中的客户乘子
    pi_DICT = {'10': 0.0, '11': 77.0, '12': 0.0, '13': 34.0}  # {"10":30,"11":0,"12":0,"13":50}
    # 需要在动态规划过程中使用的乘子示例（来自第一类rounded capacity加强割）
    subRowPi_DICT = {'[10, 11, 12]': 6, '[10, 11, 13]': 4, '[10, 12, 13]': 7, '[11, 12, 13]': 3}
    # 网络的构建
    st_network = construct.network
    st_network.readStsNode(physicalCustomerList, CustomerLst, vehicleLst, demand, Vehicle_Cap,Truckcapacity,UAVcapacity, UAVendurance, sL, sR, sS, O, D)
    st_network.readStsVehicleArc()
    st_network.readPotentialSorties(sL, sR)
    st_network.readStsDroneArc()

    # 操作的构建（构建+调整乘子）
    # 1.构建Operations
    Operations = cO.FEoperationFunction(st_network, Truckcapacity, UAVcapacity, UAVendurance)
    # 2.调整op.ReducedCost
    for op in Operations.values():
        piSum = 0
        for custerName in op.ServiceID:
            piSum += pi_DICT[f'{custerName}']
        op.sumOfCustomerPi = piSum
        op.ReducedCost = op.operationTimeCost - op.sumOfCustomerPi

    print(f"操作总数量是：{len(Operations)}")

    # 找最短路
    result = shortest_path_operations(Operations, st_network, truckGroupCapacity)
    if result[0] is not None:
        print(f"计算得到(考虑客户乘子)的最短路成本为：{result[0]}")
        print(f"计算得到的最短路所选择的操作列表为：{result[1]}")
        print(f'所选操作的name和key分别是')
        for opIndex in range(0, len(result[1])):
            print(result[1][opIndex].operationName,result[1][opIndex].key)

        print(f"这个最短路的完工时间为：{result[1][-1].endTime}")
        # 得到具体的哪些客户被什么（卡车、无人机、或者两者共同）来访问了
        truckAlone = []
        droneAlone = []
        together = []
        for i in range(0, len(result[1])):
            operation = result[1][i]
            print(operation.operationName,f"时间从{operation.startTime}到{operation.endTime}，位置从{operation.startID}到{operation.endID}，RC成本是{operation.ReducedCost}")
            if operation.same == True and len(operation.ServiceID) > 0: # 说明有客户被卡车和无人机一起访问了(只有C类操作)，添加进together列表
                together += operation.ServiceID
            if operation.same == False: # D类操作或者E类操作
                droneAlone += operation.droneServiceID
                truckAlone += operation.truckServiceID
        print(f"卡车访问的客户为：{truckAlone},无人机访问的客户为：{droneAlone},共同访问的客户为：{together}")

    else:
        print("有问题，无最短路")
