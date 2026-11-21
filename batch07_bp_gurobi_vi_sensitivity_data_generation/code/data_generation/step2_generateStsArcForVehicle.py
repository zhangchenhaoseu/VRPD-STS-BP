# -*- coding: utf-8 -*-
# Step 2: truck space-time-state arcs (STS_arc_vehicle.txt and the (1,1)-free copy).
# Original file name in the authors' project: 第2步.generateStsArcForVehicle.py (last edited 2024/10/9)
# 第2步，生成车辆的时空状态弧集合，并用文件保存
# 车辆的弧类型：0-运输弧、1-等待弧、2-发射弧、3-回收弧、4-服务弧（含车场流出和流入）
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
import pandas as pd


def generateStsArcForVehicle(customerList,sL,sR,O,D,sS,name):
    readPath_vehicle = rf'{DATA_DIR}/{name}/OD_vehicle.txt'
    readPath_drone = rf'{DATA_DIR}/{name}/OD_drone.txt'
    readPath2 = rf'{DATA_DIR}/{name}/STS_node_spaceAndTime.txt'
    savePath = rf'{DATA_DIR}/{name}/STS_arc_vehicle.txt'
    savePath2 = rf'{DATA_DIR}/{name}/STS_arc_vehicle_remove(1,1).txt'

    readDf_vehicle = pd.read_csv(readPath_vehicle)
    readDf_drone  = pd.read_csv(readPath_drone)
    readDf2 = pd.read_csv(readPath2)
    # print(readDf)
    # print(readDf2)
    notCustomerList = []
    stateList = [0, 1]
    timeStampList = list(pd.unique(readDf_vehicle['time_stamp']))
    nodeList = list(pd.unique(readDf_vehicle['from_node_id'])) + list(pd.unique(readDf_vehicle['to_node_id']))
    nodeList = list(set(nodeList))
    truckArcList = []
    droneArcList = []
    for i in range(0,len(readDf_vehicle)):
        truckArcList.append((readDf_vehicle.loc[i,'from_node_id'],readDf_vehicle.loc[i,'to_node_id']))
    truckArcList = list(set(truckArcList))
    for i in range(0,len(readDf_drone)):
        droneArcList.append((readDf_drone.loc[i,'from_node_id'],readDf_drone.loc[i,'to_node_id']))
    droneArcList = list(set(droneArcList))
    for i in nodeList:
        if i not in customerList:
            notCustomerList.append(i)

    print('stateList:', stateList)
    print('timeStampList:', timeStampList)
    print('truckArcList:', truckArcList, len(truckArcList))
    print('droneArcList:', droneArcList, len(droneArcList))
    print('nodeList:', nodeList)
    # print('notCustomerList:', notCustomerList)

    file = open(savePath, 'w').close()
    file_generate = open(savePath, mode='a')
    file_generate.write(
        str('from_space') + "," + str('to_space') + "," + str('from_time') + "," + str('to_time') + "," + str(
            'from_state') + "," + str('to_state') + "," + str('cost') + "," + str('arc_type'))  # 表头
    file_generate.write('\n')

    file2 = open(savePath2, 'w').close()
    file_generate2 = open(savePath2, mode='a')
    file_generate2.write(
        str('from_space') + "," + str('to_space') + "," + str('from_time') + "," + str('to_time') + "," + str(
            'from_state') + "," + str('to_state') + "," + str('cost') + "," + str('arc_type'))  # 表头
    file_generate2.write('\n')

    # 构建车辆运输弧
    cnt = 0
    for w in stateList:
        for arc in truckArcList:
            i = arc[0]  # 弧起点空间位置
            j = arc[1]  # 弧终点空间位置
            if (i in customerList) or (i in [O, D]) or (j in customerList) or (j in [O,D]): # 一定是服务弧
                pass
            else:
                for t in timeStampList:
                    index = readDf_vehicle[(readDf_vehicle['from_node_id'] == i) & (readDf_vehicle['to_node_id'] == j) & (readDf_vehicle['time_stamp'] == t)].index[0]
                    T_i_j_t = readDf_vehicle.loc[index, 'driving_time_cost']
                    cost = T_i_j_t  # 运输弧成本
                    if t + T_i_j_t in timeStampList:
                        indexTimeWindow_to = readDf2[(readDf2['space'] == j) & (readDf2['time'] == t + T_i_j_t)].index[0]
                        TimeWindow_to = readDf2.loc[indexTimeWindow_to, 'timeWindow']  # 当前时空终点的时间窗状态：0不可访问、1可以访问

                        if (i == j) or (t == t + T_i_j_t) or (i == D):  # 位置不发生变化的运输弧、时间不发生变化的运输弧、终点位置出发的运输弧、起终时空点任意一个没有时间窗的
                            pass
                        else:
                            file_generate.write(str(i) + "," + str(j) + "," + str(t) + "," + str(t + T_i_j_t) + "," + str(w) + "," + str(w) + "," + str(cost) + "," + str(0))
                            file_generate.write('\n')  # 写入计算结果
                            cnt += 1
    print('车辆运输弧数量:', cnt)

    # 构建车辆等待弧
    """注，最早到达问题，终点的等待弧设置为0，其余不变"""
    cnt = 0
    for w in stateList:
        for i in nodeList:
            if i not in customerList:  # 虚拟客户位置是不会有等待弧的
                for t in timeStampList:
                    cost = 1  # 单位时间切片等待弧成本
                    if t + cost in timeStampList:
                        if i == D:  # 注，最早到达问题，终点的等待弧设置为0，其余不变
                            file_generate.write(str(i) + "," + str(i) + "," + str(t) + "," + str(t + 1) + "," + str(w) + "," + str(w) + "," + str(0) + "," + str(1))
                            file_generate.write('\n')  # 写入计算结果
                            cnt += 1
                        else:
                            file_generate.write(str(i) + "," + str(i) + "," + str(t) + "," + str(t + 1) + "," + str(w) + "," + str(w) + "," + str(cost) + "," + str(1))
                            file_generate.write('\n')  # 写入计算结果
                            cnt += 1
    print('车辆等待弧数量:', cnt)

    # 构建车辆发射弧及回收弧
    cnt = 0
    for i in nodeList:
        if i in CandidateRendezvous:
            for t in timeStampList:
                costLaunch = sL  # 发射弧成本
                costRecover = sR  # 回收弧成本
                if (i in notCustomerList) and (t + sL in timeStampList)and (i not in [O,D]):  # 发射弧的起点位置不能是虚拟客户点，虚拟起终点车场
                    file_generate.write(str(i) + "," + str(i) + "," + str(t) + "," + str(t + costLaunch) + "," + str(0) + "," + str(1) + "," + str(costLaunch) + "," + str(2))  # 'from_space'，'to_space'，'from_time'，'to_time'，'from_state'，'to_state'，'cost'，'arc_type'
                    file_generate.write('\n')  # 写入计算结果
                    cnt += 1  # 发射弧：type-2
                if (i in notCustomerList) and (t + sR in timeStampList)and (i not in [O,D]):  # 回收弧的起点位置不能是虚拟客户点，虚拟起终点车场
                    file_generate.write(str(i) + "," + str(i) + "," + str(t) + "," + str(t + costRecover) + "," + str(1) + "," + str(0) + "," + str(costRecover) + "," + str(3))  # 'from_space'，'to_space'，'from_time'，'to_time'，'from_state'，'to_state'，'cost'，'arc_type'
                    file_generate.write('\n')  # 写入计算结果
                    cnt += 1  # 回收弧：type-3
        print('车辆发射弧及回收弧数量:', cnt)

    # 构建车辆服务弧
    cnt = 0
    for w in stateList:
        for arc in truckArcList:
            i = arc[0]  # 弧起点空间位置
            j = arc[1]  # 弧终点空间位置
            if (i in customerList): # 如果这条弧的起点是虚拟客户节点或者虚拟起终点车场
                for t in timeStampList:
                    cost = sS  # 服务弧成本
                    if t + cost in timeStampList:
                        file_generate.write(str(i) + "," + str(j) + "," + str(t) + "," + str(t + cost) + "," + str(w) + "," + str(w) + "," + str(cost) + "," + str(4))
                        file_generate.write('\n')  # 写入计算结果
                        cnt += 1  # 服务弧：type-4

            elif (j in customerList): # 如果这条弧的终点是虚拟客户节点或者虚拟起终点车场
                for t in timeStampList:
                    cost = sS  # 服务弧成本
                    if t + cost in timeStampList:
                        indexTimeWindow_to = readDf2[(readDf2['space'] == j) & (readDf2['time'] == t + cost)].index[0]
                        TimeWindow_to = readDf2.loc[indexTimeWindow_to, 'timeWindow']  # 当前时空终点的时间窗状态：0不可访问、1可以访问
                        if TimeWindow_to == 1:
                            file_generate.write(str(i) + "," + str(j) + "," + str(t) + "," + str(t + cost) + "," + str(w) + "," + str(w) + "," + str(cost) + "," + str(4))
                            file_generate.write('\n')  # 写入计算结果
                            cnt += 1  # 服务弧：type-4
            else:
                pass

    for w in [0]:
        for arc in truckArcList:
            i = arc[0]  # 弧起点空间位置
            j = arc[1]  # 弧终点空间位置
            if i in [O, D] and (j!=D):  # 如果这条弧的起点是虚拟起终点车场
                for t in timeStampList:
                    cost = 1  # 服务弧成本
                    if t + cost in timeStampList:
                        file_generate.write(
                            str(i) + "," + str(j) + "," + str(t) + "," + str(t + cost) + "," + str(w) + "," + str(
                                w) + "," + str(cost) + "," + str(4))
                        file_generate.write('\n')  # 写入计算结果
                        cnt += 1  # 服务弧：type-4

            if j in [O, D] and (i!=O):  # 如果这条弧的终点是虚拟客户节点或者虚拟起终点车场
                for t in timeStampList:
                    cost = sS  # 服务弧成本
                    if t + cost in timeStampList:
                        file_generate.write(str(i) + "," + str(j) + "," + str(t) + "," + str(t + cost) + "," + str(w) + "," + str(w) + "," + str(cost) + "," + str(4))
                        file_generate.write('\n')  # 写入计算结果
                        cnt += 1  # 服务弧：type-4

            if (i,j) == (O,D):  # 如果这条弧的起点是虚拟起终点车场
                for t in timeStampList:
                    cost = 0  # 服务弧成本
                    if t + cost in timeStampList:
                        file_generate.write(
                            str(i) + "," + str(j) + "," + str(t) + "," + str(t + cost) + "," + str(w) + "," + str(
                                w) + "," + str(cost) + "," + str(4))
                        file_generate.write('\n')  # 写入计算结果
                        cnt += 1  # 服务弧：type-4

            else:
                pass

    print('车辆服务弧数量:', cnt)
    file_generate.close()

    """构建排除掉状态=（1，1）的车辆运输弧"""
    # 构建车辆运输弧
    cnt = 0
    for w in [0]:
        for arc in truckArcList:
            i = arc[0]  # 弧起点空间位置
            j = arc[1]  # 弧终点空间位置
            if (i in customerList) or (i in [O, D]) or (j in customerList) or (j in [O, D]):  # 一定是服务弧
                pass
            else:
                for t in timeStampList:
                    index = readDf_vehicle[(readDf_vehicle['from_node_id'] == i) & (readDf_vehicle['to_node_id'] == j) & (
                                readDf_vehicle['time_stamp'] == t)].index[0]
                    T_i_j_t = readDf_vehicle.loc[index, 'driving_time_cost']
                    cost = T_i_j_t  # 运输弧成本
                    if t + T_i_j_t in timeStampList:
                        indexTimeWindow_to = readDf2[(readDf2['space'] == j) & (readDf2['time'] == t + T_i_j_t)].index[0]
                        TimeWindow_to = readDf2.loc[indexTimeWindow_to, 'timeWindow']  # 当前时空终点的时间窗状态：0不可访问、1可以访问

                        if (i == j) or (t == t + T_i_j_t) or (i == D):  # 位置不发生变化的运输弧、时间不发生变化的运输弧、终点位置出发的运输弧、起终时空点任意一个没有时间窗的
                            pass
                        else:
                            file_generate2.write(
                                str(i) + "," + str(j) + "," + str(t) + "," + str(t + T_i_j_t) + "," + str(w) + "," + str(
                                    w) + "," + str(cost) + "," + str(0))
                            file_generate2.write('\n')  # 写入计算结果
                            cnt += 1
    print('排除掉状态=（1，1）的车辆运输弧数量:', cnt)

    # 构建车辆等待弧
    """注，最早到达问题，终点的等待弧设置为0，其余不变"""
    cnt = 0
    for w in [0]:
        for i in nodeList:
            if i not in customerList:  # 虚拟客户位置是不会有等待弧的
                for t in timeStampList:
                    cost = 1  # 单位时间切片等待弧成本
                    if t + cost in timeStampList:
                        if i == D:  # 注，最早到达问题，终点的等待弧设置为0，其余不变
                            file_generate2.write(
                                str(i) + "," + str(i) + "," + str(t) + "," + str(t + 1) + "," + str(w) + "," + str(
                                    w) + "," + str(0) + "," + str(1))
                            file_generate2.write('\n')  # 写入计算结果
                            cnt += 1
                        else:
                            file_generate2.write(
                                str(i) + "," + str(i) + "," + str(t) + "," + str(t + 1) + "," + str(w) + "," + str(
                                    w) + "," + str(cost) + "," + str(1))
                            file_generate2.write('\n')  # 写入计算结果
                            cnt += 1
        print('排除掉状态=（1，1）的车辆等待弧数量:', cnt)

    # 构建车辆发射弧及回收弧
    cnt = 0
    for i in nodeList:
        if i in CandidateRendezvous:
            for t in timeStampList:
                costLaunch = sL  # 发射弧成本
                costRecover = sR  # 回收弧成本
                if (i in notCustomerList) and (t + sL in timeStampList)and (i not in [O,D]):  # 发射弧的起点位置不能是虚拟客户点，虚拟起终点车场
                    file_generate2.write(
                        str(i) + "," + str(i) + "," + str(t) + "," + str(t + costLaunch) + "," + str(0) + "," + str(
                            1) + "," + str(costLaunch) + "," + str(
                            2))  # 'from_space'，'to_space'，'from_time'，'to_time'，'from_state'，'to_state'，'cost'，'arc_type'
                    file_generate2.write('\n')  # 写入计算结果
                    cnt += 1  # 发射弧：type-2
                if (i in notCustomerList) and (t + sR in timeStampList)and (i not in [O,D]):  # 回收弧的起点位置不能是虚拟客户点，虚拟起终点车场
                    file_generate2.write(
                        str(i) + "," + str(i) + "," + str(t) + "," + str(t + costRecover) + "," + str(1) + "," + str(
                            0) + "," + str(costRecover) + "," + str(
                            3))  # 'from_space'，'to_space'，'from_time'，'to_time'，'from_state'，'to_state'，'cost'，'arc_type'
                    file_generate2.write('\n')  # 写入计算结果
                    cnt += 1  # 回收弧：type-3
    print('排除掉状态=（1，1）的车辆发射弧及回收弧数量:', cnt)

    # 构建车辆服务弧
    cnt = 0
    for w in [0]:
        for arc in truckArcList:
            i = arc[0]  # 弧起点空间位置
            j = arc[1]  # 弧终点空间位置
            if (i,j) != (O,D):
                if (i in customerList) or (i in [O, D]):  # 如果这条弧的起点是虚拟客户节点或者虚拟起终点车场
                    for t in timeStampList:
                        cost = 1  # 服务弧成本
                        if t + cost in timeStampList:
                            file_generate2.write(
                                str(i) + "," + str(j) + "," + str(t) + "," + str(t + cost) + "," + str(w) + "," + str(
                                    w) + "," + str(cost) + "," + str(4))
                            file_generate2.write('\n')  # 写入计算结果
                            cnt += 1  # 服务弧：type-4

                elif (j in customerList) or (j in [O, D]):  # 如果这条弧的终点是虚拟客户节点或者虚拟起终点车场
                    for t in timeStampList:
                        cost = sS  # 服务弧成本
                        if t + cost in timeStampList:
                            indexTimeWindow_to = readDf2[(readDf2['space'] == j) & (readDf2['time'] == t + cost)].index[0]
                            TimeWindow_to = readDf2.loc[indexTimeWindow_to, 'timeWindow']  # 当前时空终点的时间窗状态：0不可访问、1可以访问
                            if TimeWindow_to == 1:
                                file_generate2.write(
                                    str(i) + "," + str(j) + "," + str(t) + "," + str(t + cost) + "," + str(w) + "," + str(
                                        w) + "," + str(cost) + "," + str(4))
                                file_generate2.write('\n')  # 写入计算结果
                                cnt += 1  # 服务弧：type-4
            if (i, j) == (O, D):
                for t in timeStampList:
                    cost = 0  # 服务弧成本
                    if t + cost in timeStampList:
                        file_generate2.write(
                            str(i) + "," + str(j) + "," + str(t) + "," + str(t + cost) + "," + str(w) + "," + str(
                                w) + "," + str(cost) + "," + str(4))
                        file_generate2.write('\n')  # 写入计算结果
                        cnt += 1  # 服务弧：type-4
            else:
                pass

    print('排除掉状态=（1，1）的车辆服务弧数量:', cnt)
    file_generate2.close()


if __name__ == "__main__":
    name = "敏感性分析_SF无人机续航_第5组"
    customerList = [25,26,27,28,29,30,31,32]
    CandidateRendezvous = [14,22,4,6,23,12]

    O = 0  # 起点
    D = customerList[-1]+1 # 终点
    print("D",D)
    sL = 1  # 发射成本
    sR = 1  # 回收成本
    sS = 1  # 服务弧成本
    generateStsArcForVehicle(customerList,sL,sR,O,D,sS,name)
