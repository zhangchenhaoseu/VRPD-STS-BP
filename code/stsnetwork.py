# -*- coding: utf-8 -*-
"""Space-time-state network reader shared by all instances.

In the original project every instance had its own copy of this module (constructStsNetwork1.py,
TDconstructStsNetwork16.py, constructStsNetworkNine3.py, ...); the 67 copies differ only in the
instance name written at the top. The release keeps one copy here, and the files in
code/networks/ are two-line stubs that create ``network = ReadData(name)`` so that every driver
can keep its original ``import constructStsNetwork16 as construct16`` line.

The class bodies are those of the original modules; the only change is that the seven input
paths are instance attributes built from repo_paths.DATA_DIR instead of module-level
absolute Windows paths.

Instance folder layout (DATA_DIR/<name>/):
    STS_node.txt, STS_node_spaceAndTime.txt       space-time-state nodes, customer time windows
    STS_arc_vehicle.txt, STS_arc_drone.txt        truck and drone space-time-state arcs
    STS_arc_vehicle_remove(1,1).txt,
    STS_arc_drone_remove(1,1).txt                 the same arcs without the (1,1) state pairs
    Potential_sorties.txt                         feasible drone sorties (i, j, k, t, s, p, cost)
    OD_vehicle.txt, OD_drone.txt                  time-stamped travel times (input of step 1 to 4)
"""
import os
import pandas as pd
from itertools import combinations

from repo_paths import DATA_DIR

class StsNode:  # 定义空间-时间-状态节点类，实例为(i,t,w),(j,s,w')
    def __init__(self):
        self.space = None
        self.time = None
        self.state = None
        self.nodeType = None  # 0-实际交通网络节点、1-虚拟客户节点
        self.stsNode = None  # （i,t,w）


class StsVehicleArc:  # 定义【车辆的】空间-时间-状态弧类，实例为(i,j,t,s,w,w')
    def __init__(self):
        self.from_space = None
        self.to_space = None
        self.from_time = None
        self.to_time = None
        self.from_state = None
        self.to_state = None
        self.cost = None
        self.arcType = None  # 0-运输弧、1-等待弧、2-发射弧、3-回收弧、4-服务弧
        self.stsArc = None  # (i,j,t,s,w,w')


class StsDroneArc:  # 定义【无人机的】空间-时间-状态弧类，实例为(i,j,t,s,w,w')
    def __init__(self):
        self.from_space = None
        self.to_space = None
        self.from_time = None
        self.to_time = None
        self.from_state = None
        self.to_state = None
        self.cost = None
        self.arcType = None  # 0-运输弧、1-运输等待弧、2-发射弧、3-回收弧、4-服务弧
        self.stsArc = None  # (i,j,t,s,w,w')


class PotentialSorties:  # 定义【无人机的】空间-时间-状态潜在架次类，实例为(i,j,k,t,s,p)
    def __init__(self):
        self.start_space = None  # i
        self.delivery_space = None  # j
        self.end_space = None  # k
        self.start_time = None  # t
        self.delivery_time = None  # s
        self.end_time = None  # p
        self.sorties_cost = None  # 架次时间成本
        self.PotentialSorties = None  # (i,j,k,t,s,p)
        self.usedDroneArcList = []


class ReadData:  # 读取路网数据
    FILES = {
        'readPath_StsNodeSpaceAndTime': 'STS_node_spaceAndTime.txt',
        'readPath_StsNode': 'STS_node.txt',
        'readPath_StsArcVehicle': 'STS_arc_vehicle.txt',
        'readPath_StsArcDrone': 'STS_arc_drone.txt',
        'readPath_PotentialSorties': 'Potential_sorties.txt',
        'readPath_ArcVehicleRemoval': 'STS_arc_vehicle_remove(1,1).txt',
        'readPath_ArcDroneRemoval': 'STS_arc_drone_remove(1,1).txt',
    }

    def __init__(self, name):
        """name: instance folder under DATA_DIR, e.g. 'timeDependent_SFnetwork_16customer'."""
        self.name = name
        for attr, fname in self.FILES.items():
            setattr(self, attr, os.path.join(DATA_DIR, name, fname))

    def enumerate_subsets(self):
        """Enumerate every non-empty customer subset (allSubset) and the subsets whose demand fits
        into one group (capacitySubset). This is what readStsNode did before the rounded capacity
        inequalities were separated lazily (lazyrci.py, September 2026); the B&P-Gurobi-VI scripts in
        code/baselines/bp_gurobi_vi/ still add the inequalities a priori and therefore call it after
        readStsNode. The main B&P drivers do not need it."""
        all_subsets = []
        for r in range(1, len(self.Customer) + 1):  # 子集大小从 1 到 len(CustomerLst)，排除空集
            all_subsets.extend(combinations(self.Customer, r))
        self.allSubset = [list(subset) for subset in all_subsets]  # 客户点的所有子集
        result = []  # 符合容量约束的组合
        for _set in self.allSubset:
            totalDemand = 0
            for customer in _set:
                customerIdx = self.Customer.index(customer)
                totalDemand += self.demand[customerIdx]
            if totalDemand <= self.vehicleCap[0]:
                result.append(_set)
        self.capacitySubset = result

    def readStsNode(self,physicalCustomerList,CustomerLst,vehicleLst,demand,Vehicle_Cap,Truckcapacity,UAVcapacity, UAVendurance,sL,sR,sS,O,D):  # 针对stsNode类的实例方法
        dfStsNode = pd.read_csv(self.readPath_StsNode)
        dfStsNode1 = pd.read_csv(self.readPath_StsNodeSpaceAndTime)
        self.stsNodeList = []
        self.sL = sL
        self.sR = sR
        self.sS = sS
        self.O = O
        self.D = D
        self.Customer = CustomerLst  # 虚拟客户节点索引
        # 生成客户的所有子集
        # [lazy] 不再预先枚举 2^n 个客户子集（24 客户时超过 4 GB 内存）。rounded capacity 割改为按需分离，
        # [lazy] 见 lazyrci.py；network.allSubset 由各分支节点已加入的割列表接管。
        self.allSubset = []  # [lazy]
        self.physicalCustomer = physicalCustomerList  # 实际客户节点索引
        self.demand = demand
        self.vehicleCap = Vehicle_Cap  #卡车组容量
        self.Truckcapacity = Truckcapacity
        self.UAVcapacity = UAVcapacity
        self.UAVendurance = UAVendurance
        self.vehicleLst = vehicleLst

        self.capacitySubset = []  # [lazy]

        self.TTB = max(pd.unique(dfStsNode['time']))
        self.nodeNum = len(pd.unique(dfStsNode['space']))  # 有多少个节点
        self.timeNum = len(pd.unique(dfStsNode['time']))  # 有多少个时间切片
        self.stateNum = len(pd.unique(dfStsNode['state']))  # 有多少个状态切片

        self.Node = list(pd.unique(dfStsNode['space']))  # 这些节点都包括哪些内容
        self.T = list(pd.unique(dfStsNode['time']))  # 这些时间切片都包括哪些内容
        self.TimeWindowMtx = [[] for i in range(0,len(self.Customer))]  # 二维列表，内含各个客户节点的可行时间切片（可被访问的时间点）


        for i in range(0, len(dfStsNode)):
            n = StsNode()  # 实例化StsNode，实例为（i,t,w）
            n.space = dfStsNode.loc[i,'space']
            n.time = dfStsNode.loc[i,'time']
            n.state = dfStsNode.loc[i,'state']
            n.nodeType = dfStsNode.loc[i,'customer']
            n.stsNode = (n.space, n.time, n.state)
            self.stsNodeList.append(n)

        for i in range(0, len(dfStsNode1)):
            space = dfStsNode1.loc[i,'space']
            time = dfStsNode1.loc[i,'time']
            timeWindow = dfStsNode1.loc[i,'timeWindow']
            if (space in self.Customer) and (timeWindow == 1):  # 如果这个时空节点的空间索引是客户点
                customerIndex = self.Customer.index(space)  # 定位
                self.TimeWindowMtx[customerIndex].append(time)  # 插入可行时间切片
        print('不同虚拟客户点的时间窗:',self.TimeWindowMtx)


    def readStsVehicleArc(self):  # 针对车辆的StsVehicleArc类的实例方法
        dfStsVehicleArc = pd.read_csv(self.readPath_StsArcVehicle)
        dfStsVehicleArcRemoval = pd.read_csv(self.readPath_ArcVehicleRemoval)
        self.stsVehicleArcList = []
        self.stsVehicleArcRemovalList = []  # Bx去除（1，1）
        for i in range(0, len(dfStsVehicleArc)):
            a = StsVehicleArc()  # 实例化StsVehicleArc，实例为(i,j,t,s,w,w')
            a.from_space = dfStsVehicleArc.loc[i,'from_space']  # i
            a.to_space = dfStsVehicleArc.loc[i,'to_space']  # j
            a.from_time = dfStsVehicleArc.loc[i,'from_time']  # t
            a.to_time = dfStsVehicleArc.loc[i,'to_time']  # s
            a.from_state = dfStsVehicleArc.loc[i,'from_state']  # w
            a.to_state = dfStsVehicleArc.loc[i,'to_state']  # w'
            a.cost = dfStsVehicleArc.loc[i,'cost']
            a.arcType = dfStsVehicleArc.loc[i,'arc_type']
            a.stsArc = (a.from_space,a.to_space,a.from_time,a.to_time,a.from_state,a.to_state)  # (i,j,t,s,w,w')
            self.stsVehicleArcList.append(a)

        for i in range(0,len(dfStsVehicleArcRemoval)):
            a = StsVehicleArc()  # 实例化StsVehicleArc，实例为(i,j,t,s,w,w')
            a.from_space = dfStsVehicleArcRemoval.loc[i, 'from_space']
            a.to_space = dfStsVehicleArcRemoval.loc[i, 'to_space']
            a.from_time = dfStsVehicleArcRemoval.loc[i, 'from_time']
            a.to_time = dfStsVehicleArcRemoval.loc[i, 'to_time']
            a.from_state = dfStsVehicleArcRemoval.loc[i, 'from_state']
            a.to_state = dfStsVehicleArcRemoval.loc[i, 'to_state']
            a.cost = dfStsVehicleArcRemoval.loc[i, 'cost']
            a.arcType = dfStsVehicleArcRemoval.loc[i, 'arc_type']
            a.stsArc = (a.from_space, a.to_space, a.from_time, a.to_time, a.from_state, a.to_state)  # (i,j,t,s,w,w')
            self.stsVehicleArcRemovalList.append(a)
    def readPotentialSorties(self,sL,sR):  # 针对无人机的PotentialSorties类的实例方法
        dfPotentialSorties = pd.read_csv(self.readPath_PotentialSorties)
        self.PotentialSortiesList = []
        for i in range(0, len(dfPotentialSorties)):
            c = PotentialSorties()  # 实例化PotentialSorties，实例为(i,j,k,t,s,p)
            c.start_space = dfPotentialSorties.loc[i,'start_space'] # i
            c.delivery_space = dfPotentialSorties.loc[i,'delivery_space'] # j
            c.end_space = dfPotentialSorties.loc[i,'end_space'] # k
            c.start_time = dfPotentialSorties.loc[i,'start_time']  # t
            c.delivery_time = dfPotentialSorties.loc[i,'delivery_time'] # s
            c.end_time = dfPotentialSorties.loc[i,'end_time']  # p
            c.sorties_cost = dfPotentialSorties.loc[i,'sorties_cost']  # 架次时间成本
            c.PotentialSorties = (c.start_space,c.delivery_space,c.end_space,c.start_time,c.delivery_time,c.end_time)  # (i,j,k,t,s,p)
            c.usedDroneArcList.append((c.start_space,c.start_space,c.start_time-sL,c.start_time,0, 1))
            c.usedDroneArcList.append((c.start_space, c.delivery_space, c.start_time, c.delivery_time, 1, 1))
            c.usedDroneArcList.append((c.delivery_space, c.end_space, c.delivery_time, c.end_time, 1, 1))
            c.usedDroneArcList.append((c.end_space, c.end_space, c.end_time, c.end_time+sR, 1, 0))
            self.PotentialSortiesList.append(c)

    def readStsDroneArc(self):  # 针对无人机的StsDroneArc类的实例方法
        dfStsDroneArc = pd.read_csv(self.readPath_StsArcDrone)
        dfStsDroneArcRemoval = pd.read_csv(self.readPath_ArcDroneRemoval)
        self.stsDroneArcList = []
        self.stsDroneArcRemovalList = []  # By去除（1，1），这两个是一样的
        for i in range(0, len(dfStsDroneArc)):
            b = StsDroneArc()  # 实例化StsDroneArc，实例为(i,j,t,s,w,w')
            b.from_space = dfStsDroneArc.loc[i,'from_space']
            b.to_space = dfStsDroneArc.loc[i,'to_space']
            b.from_time = dfStsDroneArc.loc[i,'from_time']
            b.to_time = dfStsDroneArc.loc[i,'to_time']
            b.from_state = dfStsDroneArc.loc[i,'from_state']
            b.to_state = dfStsDroneArc.loc[i,'to_state']
            b.cost = dfStsDroneArc.loc[i,'cost']
            b.arcType = dfStsDroneArc.loc[i,'arc_type']
            b.stsArc = (b.from_space,b.to_space,b.from_time,b.to_time,b.from_state,b.to_state)  # (i,j,t,s,w,w')
            self.stsDroneArcList.append(b)
        for i in range(0,len(dfStsDroneArcRemoval)):
            b = StsDroneArc()  # 实例化StsDroneArc，实例为(i,j,t,s,w,w')
            b.from_space = dfStsDroneArcRemoval.loc[i, 'from_space']
            b.to_space = dfStsDroneArcRemoval.loc[i, 'to_space']
            b.from_time = dfStsDroneArcRemoval.loc[i, 'from_time']
            b.to_time = dfStsDroneArcRemoval.loc[i, 'to_time']
            b.from_state = dfStsDroneArcRemoval.loc[i, 'from_state']
            b.to_state = dfStsDroneArcRemoval.loc[i, 'to_state']
            b.cost = dfStsDroneArcRemoval.loc[i, 'cost']
            b.arcType = dfStsDroneArcRemoval.loc[i, 'arc_type']
            b.stsArc = (b.from_space, b.to_space, b.from_time, b.to_time, b.from_state, b.to_state)  # (i,j,t,s,w,w')
            self.stsDroneArcRemovalList.append(b)
