# -*- coding: utf-8 -*-
# Step 4: feasible drone sorties with the payload-dependent endurance test (Potential_sorties.txt).
# Original file name in the authors' project: 第4步.generatePotentialSorties.py (last edited 2024/10/22)
# 第4步，生成无人机的可行sortie集合，并使用文件保存，在这里可以加入无人机不能进行服务的约束、容量、续航等约束
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
import pandas as pd


def generatePotentialSorties(physicalCustomerList,customerList,demand,UAV_Cap, f,sL,sR,TTB,O,D,name):
    readPathArc = rf'{DATA_DIR}/{name}/STS_arc_drone.txt'
    savePath = rf'{DATA_DIR}/{name}/Potential_sorties.txt'
    readPath_StsNodeSpaceAndTime = rf'{DATA_DIR}/{name}/STS_node_spaceAndTime.txt'
    readDfArc = pd.read_csv(readPathArc)
    file = open(savePath, 'w').close()
    file_generate = open(savePath, mode='a')
    file_generate.write(str('start_space') + ","+str('delivery_space')+ "," +str('end_space')+ "," +str('start_time') + "," +str('delivery_time')+ "," +str('end_time')+","+str('sorties_cost'))  # 表头
    file_generate.write('\n')

    dfStsNode = pd.read_csv(readPath_StsNodeSpaceAndTime)
    stsCustomerWindowNodeLst = []
    # print(dfStsNode)
    for i in range(0,len(dfStsNode)):
        j_name = dfStsNode.loc[i,'space']
        s_name = dfStsNode.loc[i,'time']
        timeWindow = dfStsNode.loc[i, 'timeWindow']
        if (j_name in customerList) and (timeWindow == 1):
            stsCustomerWindowNodeLst.append((j_name,s_name))  # 这里添加的是虚拟客户点
    # print('stsCustomerWindowNodeLst',stsCustomerWindowNodeLst)
    # 先将符合条件的无人机弧筛选出来，再构建架次
    StsDroneArcList = []  # 状态为（1，1）的，且为运输弧、运输等待弧的无人机时空弧
    for i in range(0,len(readDfArc)):
        from_space = readDfArc.loc[i,'from_space']
        to_space = readDfArc.loc[i,'to_space']
        from_time = readDfArc.loc[i,'from_time']
        to_time = readDfArc.loc[i,'to_time']
        from_state = readDfArc.loc[i,'from_state']
        to_state = readDfArc.loc[i,'to_state']
        arc_cost = readDfArc.loc[i,'cost']
        arc_type = readDfArc.loc[i,'arc_type']
        if (from_state == 1) and (to_state == 1) and (arc_type == 0 or arc_type == 1):
            StsDroneArcList.append((from_space,to_space,from_time,to_time,arc_cost))
    # print('StsDroneArcList:',StsDroneArcList)

    cnt =0
    # 构建符合条件的架次
    for flyInArc in StsDroneArcList:  # 流入弧
        if (flyInArc[2] != TTB) and (flyInArc[2] != 0) and (flyInArc[0] in CandidateRendezvous) and (flyInArc[3] != TTB) and (flyInArc[1] in physicalCustomerList) and(flyInArc[0]!=flyInArc[1]) and((customerList[physicalCustomerList.index(flyInArc[1])], flyInArc[3]+sS) in stsCustomerWindowNodeLst) and(flyInArc[0] != D):
            # 对于飞行流入弧，要求其起点时刻不等于TTB and 不等于0 and 起点位置是潜在起降点（起点位置是实际交通网络物理节点） and 终点时刻不等于TTB and 终点位置必须是实际客户点(对流入弧的要求) and i!=j and （实际终点位置对应的虚拟节点，到达实际终点位置的时刻+服务时长）在客户时间窗内 and 起点位置不是终点位置
            # 上面筛选出了流入弧，但实际上这只是架次前段的其中一部分，应当把服务弧的信息加上！
            flyIn_start_space = flyInArc[0]  # 这个流入弧的起点位置i
            flyIn_end_space = flyInArc[1]  # 这个流入弧的终点位置j
            flyIn_start_time = flyInArc[2]  # 这个流入弧的起点时刻t
            flyIn_end_time = flyInArc[3]  # 这个流入弧的终点时刻s
            flyIn_arc_cost = flyInArc[4]  # 这个流入弧的成本（时耗）

            sortie_start_space = flyIn_start_space  # 待生成架次的起点位置
            sortie_start_time = flyIn_start_time  # 待生成架次的起点时刻

            sortie_mid_space = customerList[physicalCustomerList.index(flyIn_end_space)]  # 待生成架次的中间点位置
            d = demand[physicalCustomerList.index(flyIn_end_space)]  # 对应这个中间点的需求量
            sortie_mid_time = flyIn_end_time + sS  # 待生成架次的中间点时刻

            for flyOutArc in StsDroneArcList:
                if (flyInArc != flyOutArc) and (flyOutArc[0] == flyIn_end_space) and (flyOutArc[2] == flyIn_end_time+2*sS) and (flyOutArc[3] !=0) and (flyOutArc[1] in CandidateRendezvous) and(flyOutArc[1] != flyIn_end_space) and(flyIn_arc_cost+flyOutArc[4]<=f-sR-sL-2*sS-d) and(flyOutArc[3]<= TTB-1-sR) and (d<=UAV_Cap): # 1是要回车场

                    # 流入和流出不能是同一条弧,同时流出弧和流入弧要衔接起来（对衔接的要求）
                    # 对于飞行流出弧，起点位置一定是客户点（衔接保证了）、起点时刻不等于TTB（衔接保证了）、终点时刻不等于0 and 终点位置是潜在起降点 and(k!=j) and 总时耗不超过滞空时长 and 流出弧的终点时刻小于等于TTB-1 and 服务量小于无人机容量 and 【无人机的飞行续航收到载荷影响，这里是简单的作差线性关系】
                    flyOut_start_space = flyOutArc[0]  # 这个流出弧的起点位置j
                    flyOut_end_space = flyOutArc[1]  # 这个流出弧的终点位置k
                    flyOut_start_time = flyOutArc[2]  # 这个流出弧的起点时刻s
                    flyOut_end_time = flyOutArc[3]  # 这个流出弧的终点时刻p
                    flyOut_arc_cost = flyOutArc[4]  # 这个流出弧的成本（时耗）

                    sortie_end_space = flyOut_end_space  # 待生成架次的终点位置
                    sortie_end_time = flyOut_end_time  # 待生成架次的终点时刻

                    sorties_cost = flyIn_arc_cost+flyOut_arc_cost + 2*sS  # 没有计入发射、回收、但计入了中间的服务弧的成本
                    cnt += 1
                    # print('潜在架次（i,j,k,t,s,p）为：',(sortie_start_space, sortie_mid_space,sortie_end_space,sortie_start_time,sortie_mid_time,sortie_end_time),'架次时耗为：',sorties_cost)
                    # 存入文件
                    file_generate.write(str(sortie_start_space) + "," + str(sortie_mid_space) + "," + str(sortie_end_space) + "," + str(sortie_start_time) + "," + str(sortie_mid_time) + "," + str(sortie_end_time) + "," + str(sorties_cost))
                    file_generate.write('\n')  # 写入计算结果

    file_generate.close()
    print(f"共有{cnt}个潜在架次")


if __name__ == "__main__":
    name = "敏感性分析_SF无人机续航_第5组"
    customerList = [25, 26, 27, 28, 29, 30, 31, 32]
    physicalCustomerList = [1, 2, 9, 12, 14, 16, 20, 23]  # 实际的客户点
    demand = [4, 7, 3, 5, 6, 3, 6, 4]  # 虚拟客户点对应的需求量
    CandidateRendezvous = [14, 22, 4, 6, 23, 12]  # 潜在起降点
    TTB = 60
    O = 0 #起点
    D = 33 #终点
    UAV_Cap = 5  # 无人机的容量
    f = 15  # 无人机飞行续航时间
    sL = 1 # 发射成本
    sR = 1 # 回收成本
    sS = 1 # 服务时长

    generatePotentialSorties(physicalCustomerList, customerList, demand, UAV_Cap,f, sL, sR, TTB, O, D,name)
