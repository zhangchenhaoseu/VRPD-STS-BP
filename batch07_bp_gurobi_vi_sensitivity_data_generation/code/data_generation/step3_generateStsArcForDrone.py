# -*- coding: utf-8 -*-
# Step 3: drone space-time-state arcs (STS_arc_drone.txt and the (1,1)-free copy).
# Original file name in the authors' project: 第3步.generateStsArcForDrone.py (last edited 2024/10/9)
# 第3步，生成无人机的时空状态弧集合，并用文件保存
# 无人机的弧类型：0-运输弧、1-运输等待弧、2-发射弧、3-回收弧、4-服务弧（含车场流出和流入）
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
import pandas as pd


def generateStsArcForDrone(physicalCustomerList,customerList,demand,UAV_Cap,f,sL,sR,O,D,sS,name):
    readPathDrone = rf'{DATA_DIR}/{name}/OD_drone.txt'
    readPathVehicle = rf'{DATA_DIR}/{name}/OD_vehicle.txt'
    readPath2 = rf'{DATA_DIR}/{name}/STS_node_spaceAndTime.txt'
    savePath = rf'{DATA_DIR}/{name}/STS_arc_drone.txt'
    savePath2 = rf'{DATA_DIR}/{name}/STS_arc_drone_remove(1,1).txt'

    readDfDrone = pd.read_csv(readPathDrone)
    readDfVehicle = pd.read_csv(readPathVehicle)
    readDf2 = pd.read_csv(readPath2)
    # print(readDfDrone)
    notCustomerList = []
    stateList = [0, 1]
    timeStampList = list(pd.unique(readDfDrone['time_stamp']))
    TTB = max(timeStampList)
    nodeList = list(pd.unique(readDfDrone['from_node_id']))+list(pd.unique(readDfDrone['to_node_id']))
    nodeList = list(set(nodeList))

    droneArcList = []
    for i in range(0,len(readDfDrone)):
        droneArcList.append((readDfDrone.loc[i,'from_node_id'],readDfDrone.loc[i,'to_node_id']))
    droneArcList = list(set(droneArcList))

    truckArcList = []
    for i in range(0, len(readDfVehicle)):
        truckArcList.append((readDfVehicle.loc[i, 'from_node_id'], readDfVehicle.loc[i, 'to_node_id']))
    truckArcList = list(set(truckArcList))
    for i in nodeList:
        if i not in customerList:
            notCustomerList.append(i)
    print('stateList:', stateList)
    print('timeStampList:', timeStampList)
    print('droneArcList:', droneArcList, len(droneArcList))
    print('nodeList:', nodeList)
    print('notCustomerList:', notCustomerList)

    file = open(savePath, 'w').close()
    file_generate = open(savePath, mode='a')
    file_generate.write(str('from_space') + ","+str('to_space')+ "," +str('from_time')+ "," +str('to_time') + "," +str('from_state')+ "," +str('to_state')+","+str('cost')+","+str('arc_type'))  # 表头
    file_generate.write('\n')

    file2 = open(savePath2, 'w').close()
    file_generate2 = open(savePath2, mode='a')
    file_generate2.write(str('from_space') + ","+str('to_space')+ "," +str('from_time')+ "," +str('to_time') + "," +str('from_state')+ "," +str('to_state')+","+str('cost')+","+str('arc_type'))  # 表头
    file_generate2.write('\n')


    cnt=0
    # 构建无人机运输弧(飞行的+合体的)
    for arc in droneArcList: # 飞行
        i = arc[0]
        j = arc[1]
        if (i in customerList) or (i in [O, D]) or (j in customerList) or (j in [O, D]):  # 一定是服务弧或者超重的
            pass
        else:
            for t in timeStampList:
                indexDrone = readDfDrone[(readDfDrone['from_node_id'] ==i) & (readDfDrone['to_node_id']== j)& (readDfDrone['time_stamp']== t)].index[0]
                T_i_j_t_Drone = int(factor*readDfDrone.loc[indexDrone, 'flying_time_cost'])
                costDrone = T_i_j_t_Drone  # 无人机飞行运输弧成本

                if t+costDrone in timeStampList:
                    if (i==j) or (t==t+costDrone) or (i==D):  # 无需设置从自身到自身的、时间不发生变化的运输弧
                        pass
                    else:
                        file_generate.write(str(i) + "," + str(j) + "," + str(t) + "," + str(t+costDrone) + "," + str(1) + "," + str(1)+ "," + str(costDrone)+","+str(0))
                        file_generate.write('\n')  # 写入计算结果
                        cnt+=1
    for arc in truckArcList:  # 合体
        i = arc[0]
        j = arc[1]
        if (i in customerList) or (i in [O, D]) or (j in customerList) or (j in [O, D]):  # 一定是服务弧
            pass
        else:
            for t in timeStampList:
                indexVehicle = readDfVehicle[(readDfVehicle['from_node_id'] == i) & (readDfVehicle['to_node_id'] == j) & (readDfVehicle['time_stamp'] == t)].index[0]
                T_i_j_t_Vehicle = readDfVehicle.loc[indexVehicle, 'driving_time_cost']
                costVehicle = T_i_j_t_Vehicle  # 无人机合体的运输弧成本

                if t + costVehicle in timeStampList:

                    if (i==j) or(t == t+costVehicle) or (i==D):  # 位置不发生变化的运输弧、时间不发生变化的运输弧
                        pass
                    else:
                        file_generate.write(str(i) + "," + str(j) + "," + str(t) + "," + str(t + costVehicle) + "," + str(0) + "," + str(0) + "," + str(costVehicle)+","+str(0))  # 无人机合体的运输弧成本是costVehicle（时间流逝）
                        file_generate.write('\n')  # 写入计算结果
                        cnt += 1
    print("无人机运输弧数量：", cnt)

    # 构建无人机飞行的“运输等待“弧，和无人机被载运时的“等待“弧
    cnt = 0
    # 构建运输等待弧，弧的成本= 飞行成本+等待成本
    for arc in droneArcList:
        i = arc[0]
        j = arc[1]
        if (i in customerList) or (i in [O, D]) or (j in customerList) or (j in [O, D]):  # 一定是服务弧
            pass
        else:
            for t in timeStampList:
                indexDrone = readDfDrone[(readDfDrone['from_node_id'] == i) & (readDfDrone['to_node_id'] == j) & (readDfDrone['time_stamp'] == t)].index[0]
                T_i_j_t_Drone = int(factor*readDfDrone.loc[indexDrone, 'flying_time_cost'])
                costDrone = T_i_j_t_Drone  # 飞行成本

                if (t + costDrone in timeStampList) and (t+costDrone != TTB):
                    for deltaT in [delta+1 for delta in range(0,TTB-(t+costDrone))]:
                        if costDrone + deltaT <= f-sL-sR-1-2*sS:  # 这里的1是考虑到sortie由两个分支构成，每个分支肯定不可能等于f-sL-sR-2*sS，不然另一个就是0了
                            waitDrone = deltaT # 等待成本
                            if (i==j) or (t == t + costDrone + deltaT) or (i==D):  # 无需设置位置没有发生变化的运输等待弧、时间没有发生变化的运输等待弧
                                pass
                            else:
                                file_generate.write(str(i) + "," + str(j) + "," + str(t) + "," + str(t + T_i_j_t_Drone + deltaT) + "," + str(1) + "," + str(1) + "," + str(costDrone+waitDrone)+","+str(1))
                                file_generate.write('\n')  # 写入计算结果
                                cnt += 1
    print("无人机飞行状态”运输等待“弧数量：", cnt)

    cnt=0
    """注，最早到达问题，终点的等待弧设置为0，其余不变"""
    # 无人机被载运时的“等待“弧
    for i in nodeList:
        if i not in customerList: # 虚拟客户位置是不会有等待弧的
            for t in timeStampList:
                cost = 1
                if t + cost in timeStampList:
                    if i == D:
                        file_generate.write(str(i) + "," + str(i) + "," + str(t) + "," + str(t + cost) + "," + str(0) + "," + str(0) + "," + str(0) + "," + str(1))  # 无人机在终点空间位置处的等待弧成本=0
                        file_generate.write('\n')  # 写入计算结果
                        cnt += 1
                    else:
                        file_generate.write(str(i) + "," + str(i) + "," + str(t) + "," + str(t + cost) + "," + str(0) + "," + str(0) + "," + str(cost)+","+str(1)) # 无人机在非终点空间位置处合体等待时，单位时间切片等待弧成本cost=1
                        file_generate.write('\n')  # 写入计算结果
                        cnt += 1
    print("无人机合体状态”等待“弧数量：",cnt)

    # 构建无人机发射弧及回收弧
    cnt=0
    for i in nodeList:
        if i in CandidateRendezvous:
            for t in timeStampList:
                costLaunch = sL  # 发射弧成本
                costRecover = sR  # 回收弧成本
                if (i in notCustomerList) and (t + sL in timeStampList) and (i not in [O,D]):  # 发射弧的起点位置不能是客户点,虚拟起终点车场
                    file_generate.write(str(i) + "," + str(i) + "," + str(t) + "," + str(t + costLaunch) + "," + str(0) + "," + str(1) + "," + str(costLaunch)+","+str(2))  # 'from_space'，'to_space'，'from_time'，'to_time'，'from_state'，'to_state'，'cost'，'arc_type'
                    file_generate.write('\n')  # 写入计算结果
                    cnt += 1  # 发射弧：type-2
                if (i in notCustomerList) and (t + sR in timeStampList) and (i not in [O,D]):  # 回收弧的起点位置不能是客户点,虚拟起终点车场
                    file_generate.write(str(i) + "," + str(i) + "," + str(t) + "," + str(t + costRecover) + "," + str(1) + "," + str(0) + "," + str(costRecover)+","+str(3))  # 'from_space'，'to_space'，'from_time'，'to_time'，'from_state'，'to_state'，'cost'，'arc_type'
                    file_generate.write('\n')  # 写入计算结果
                    cnt += 1  # 回收弧：type-3
    print('无人机发射弧及回收弧数量:',cnt)

    # 构建无人机服务弧(飞行的+合体的)
    cnt = 0
    for arc in droneArcList:  # 飞行的
        i = arc[0]
        j = arc[1]
        if (i in customerList and demand[customerList.index(i)] <= UAV_Cap) :  # 如果这条弧的起点是虚拟客户节点或者虚拟起终点车场
            for t in timeStampList:
                costDrone = sS  # 无人机服务成本
                if t + costDrone in timeStampList:
                        file_generate.write(str(i) + "," + str(j) + "," + str(t) + "," + str(t + costDrone) + "," + str(1) + "," + str(1) + "," + str(costDrone) + "," + str(4))
                        file_generate.write('\n')  # 写入计算结果
                        cnt += 1
        elif (j in customerList and demand[customerList.index(j)] <= UAV_Cap):  # 如果这条弧的终点是虚拟客户节点或者虚拟起终点车场
            for t in timeStampList:
                costDrone = sS  # 无人机服务成本
                if t + costDrone in timeStampList:
                    indexTimeWindow_to = readDf2[(readDf2['space'] == j) & (readDf2['time'] == t + costDrone)].index[0]
                    TimeWindow_to = readDf2.loc[indexTimeWindow_to, 'timeWindow']  # 当前时空终点的时间窗状态：0不可访问、1可以访问
                    if TimeWindow_to == 1:
                        file_generate.write(str(i) + "," + str(j) + "," + str(t) + "," + str(t + costDrone) + "," + str(1) + "," + str(1) + "," + str(costDrone) + "," + str(4))
                        file_generate.write('\n')  # 写入计算结果
                        cnt += 1  # 服务弧：type-4
        else:
            pass

    for arc in truckArcList:  # 合体的
        i = arc[0]
        j = arc[1]
        if (i,j) != (O,D):
            if (i in customerList) or (i in [O, D]):  # 如果这条弧的起点是虚拟客户节点或者虚拟起终点车场
                for t in timeStampList:
                    costVehicle = sS  # 服务弧成本
                    if t + costVehicle in timeStampList:
                        file_generate.write(str(i) + "," + str(j) + "," + str(t) + "," + str(t + costVehicle) + "," + str(0) + "," + str(0) + "," + str(costVehicle) + "," + str(4))  # 无人机合体的运输弧成本是costVehicle（时间流逝）
                        file_generate.write('\n')  # 写入计算结果
                        cnt += 1
            elif (j in customerList) or (j in [O, D]):  # 如果这条弧的终点是虚拟客户节点或者虚拟起终点车场
                for t in timeStampList:
                    cost = sS  # 服务弧成本
                    if t + cost in timeStampList:
                        indexTimeWindow_to = readDf2[(readDf2['space'] == j) & (readDf2['time'] == t + cost)].index[0]
                        TimeWindow_to = readDf2.loc[indexTimeWindow_to, 'timeWindow']  # 当前时空终点的时间窗状态：0不可访问、1可以访问
                        if TimeWindow_to == 1:
                            file_generate.write(str(i) + "," + str(j) + "," + str(t) + "," + str(t + cost) + "," + str(0) + "," + str(0) + "," + str(cost) + "," + str(4))
                            file_generate.write('\n')  # 写入计算结果
                            cnt += 1  # 服务弧：type-4
        if (i,j) == (O,D):
            for t in timeStampList:
                costVehicle = 0  # 服务弧成本
                if t + costVehicle in timeStampList:
                    file_generate.write(
                        str(i) + "," + str(j) + "," + str(t) + "," + str(t + costVehicle) + "," + str(0) + "," + str(
                            0) + "," + str(costVehicle) + "," + str(4))  # 无人机合体的运输弧成本是costVehicle（时间流逝）
                    file_generate.write('\n')  # 写入计算结果
                    cnt += 1

        else:
            pass
    print("无人机服务弧数量：", cnt)

    file_generate.close()

    """构建排除掉状态=（1，1）的无人机运输弧"""
    cnt = 0
    # 构建除掉状态=（1，1）的无人机运输弧(合体的)
    for arc in truckArcList:
        i = arc[0]
        j = arc[1]
        if (i,j) != (O,D):
            if (i in customerList) or (i in [O, D]) or (j in customerList) or (j in [O, D]):  # 一定是服务弧
                pass
            else:
                for t in timeStampList:
                    indexVehicle = readDfVehicle[
                        (readDfVehicle['from_node_id'] == i) & (readDfVehicle['to_node_id'] == j) & (
                                    readDfVehicle['time_stamp'] == t)].index[0]
                    T_i_j_t_Vehicle = readDfVehicle.loc[indexVehicle, 'driving_time_cost']
                    costVehicle = T_i_j_t_Vehicle  # 无人机合体的运输弧成本

                    if t + costVehicle in timeStampList:

                        if (i == j) or (t == t + costVehicle) or (i == D):  # 位置不发生变化的运输弧、时间不发生变化的运输弧
                            pass
                        else:
                            file_generate2.write(
                                str(i) + "," + str(j) + "," + str(t) + "," + str(t + costVehicle) + "," + str(
                                    0) + "," + str(0) + "," + str(costVehicle) + "," + str(
                                    0))  # 无人机合体的运输弧成本是costVehicle（时间流逝）
                            file_generate2.write('\n')  # 写入计算结果
                            cnt += 1
        if (i, j) == (O, D):
            for t in timeStampList:
                costVehicle = 0  # 服务弧成本
                if t + costVehicle in timeStampList:
                    file_generate2.write(
                        str(i) + "," + str(j) + "," + str(t) + "," + str(t + costVehicle) + "," + str(0) + "," + str(
                            0) + "," + str(costVehicle) + "," + str(4))  # 无人机合体的运输弧成本是costVehicle（时间流逝）
                    file_generate2.write('\n')  # 写入计算结果
                    cnt += 1
    print("排除掉状态=（1，1）的无人机运输弧数量：", cnt)

    cnt = 0
    """注，最早到达问题，终点的等待弧设置为0，其余不变"""
    # 无人机被载运时的“等待“弧
    for i in nodeList:
        if i not in customerList:  # 虚拟客户位置是不会有等待弧的
            for t in timeStampList:
                cost = 1
                if t + cost in timeStampList:
                    if i == D:
                        file_generate2.write(
                            str(i) + "," + str(i) + "," + str(t) + "," + str(t + cost) + "," + str(0) + "," + str(
                                0) + "," + str(0) + "," + str(1))  # 无人机在终点空间位置处的等待弧成本=0
                        file_generate2.write('\n')  # 写入计算结果
                        cnt += 1
                    else:
                        file_generate2.write(
                            str(i) + "," + str(i) + "," + str(t) + "," + str(t + cost) + "," + str(0) + "," + str(
                                0) + "," + str(cost) + "," + str(1))  # 无人机在非终点空间位置处合体等待时，单位时间切片等待弧成本cost=1
                        file_generate2.write('\n')  # 写入计算结果
                        cnt += 1
    print("排除掉状态=（1，1）的无人机合体状态”等待“弧数量：", cnt)

    # 构建无人机发射弧及回收弧
    cnt = 0
    for i in nodeList:
        if i in CandidateRendezvous:
            for t in timeStampList:
                costLaunch = sL  # 发射弧成本
                costRecover = sR  # 回收弧成本
                if (i in notCustomerList) and (t + sL in timeStampList) and (i not in [O,D]):  # 发射弧的起点位置不能是客户点，虚拟起终点车场
                    file_generate2.write(
                        str(i) + "," + str(i) + "," + str(t) + "," + str(t + costLaunch) + "," + str(0) + "," + str(
                            1) + "," + str(costLaunch) + "," + str(
                            2))  # 'from_space'，'to_space'，'from_time'，'to_time'，'from_state'，'to_state'，'cost'，'arc_type'
                    file_generate2.write('\n')  # 写入计算结果
                    cnt += 1  # 发射弧：type-2
                if (i in notCustomerList) and (t + sR in timeStampList) and (i not in [O,D]):  # 回收弧的起点位置不能是客户点，虚拟起终点车场
                    file_generate2.write(
                        str(i) + "," + str(i) + "," + str(t) + "," + str(t + costRecover) + "," + str(1) + "," + str(
                            0) + "," + str(costRecover) + "," + str(
                            3))  # 'from_space'，'to_space'，'from_time'，'to_time'，'from_state'，'to_state'，'cost'，'arc_type'
                    file_generate2.write('\n')  # 写入计算结果
                    cnt += 1  # 回收弧：type-3
    print('排除掉状态=（1，1）的无人机发射弧及回收弧数量:', cnt)

    # 构建无人机服务弧(合体的)
    cnt = 0
    for arc in truckArcList:  # 合体的
        i = arc[0]
        j = arc[1]
        if (i in customerList) or (i in [O, D]):  # 如果这条弧的起点是虚拟客户节点或者虚拟起终点车场
            for t in timeStampList:
                costVehicle = sS  # 服务弧成本
                if t + costVehicle in timeStampList:
                    file_generate2.write(
                        str(i) + "," + str(j) + "," + str(t) + "," + str(t + costVehicle) + "," + str(0) + "," + str(
                            0) + "," + str(costVehicle) + "," + str(4))  # 无人机合体的运输弧成本是costVehicle（时间流逝）
                    file_generate2.write('\n')  # 写入计算结果
                    cnt += 1
        elif (j in customerList) or (j in [O, D]):  # 如果这条弧的终点是虚拟客户节点或者虚拟起终点车场
            for t in timeStampList:
                cost = sS  # 服务弧成本
                if t + cost in timeStampList:
                    indexTimeWindow_to = readDf2[(readDf2['space'] == j) & (readDf2['time'] == t + cost)].index[0]
                    TimeWindow_to = readDf2.loc[indexTimeWindow_to, 'timeWindow']  # 当前时空终点的时间窗状态：0不可访问、1可以访问
                    if TimeWindow_to == 1:
                        file_generate2.write(
                            str(i) + "," + str(j) + "," + str(t) + "," + str(t + cost) + "," + str(0) + "," + str(
                                0) + "," + str(cost) + "," + str(4))
                        file_generate2.write('\n')  # 写入计算结果
                        cnt += 1  # 服务弧：type-4
        else:
            pass
    print("排除掉状态=（1，1）的无人机服务弧数量：", cnt)
    file_generate2.close()


if __name__ == "__main__":
    name = "敏感性分析_SF无人机续航_第5组"
    customerList = [25, 26, 27,28,29,30,31,32]
    physicalCustomerList = [1, 2, 9, 12, 14, 16, 20, 23]  # 实际的客户点
    demand = [4,7,3,5,6,3,6,4]  # 虚拟客户点对应的需求量
    CandidateRendezvous = [14,22,4,6,23,12] # 潜在起降点
    f = 15  # 无人机飞行续航时间
    factor = 0.44444  # 卡车速度/无人机速度【注意改这个！！！！】
    sL = 1 # 发射成本
    sR = 1 # 回收成本
    sS = 1 # 服务弧成本
    O = 0 # 起点
    D = 33 # 终点
    UAV_Cap = 5  # 无人机的容量
    # Vehicle_Cap = 25  # 卡车组容量
    generateStsArcForDrone(physicalCustomerList,customerList,demand,UAV_Cap, f, sL, sR, O, D,sS,name)
