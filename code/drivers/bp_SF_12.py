# -*- coding: utf-8 -*-
# Branch-and-price driver for instance SFnetwork_12customer: blocks (1) B&P-DP, (2) B&P-DP-VI, (3) B&P-DP-VI-P (Tables 6 to 10).
# Original file name in the authors' project: 实例计算-SF的12节点（组件加速效果对比）.py (last edited 2025/3/11)
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
import multiprocessing as mp
from gurobipy import *
import constructStsNetwork12 as construct12
import time
import copy
import numpy as np
import constructOperation as cO
import DPforSPP_operation_fast as dp_SPP
import DPforSPP_operationValidCut_fast as dp_SPPVC
import DPforSPP_operationValidCut_parallel_fast as dp_SPPVC_p
import pandas as pd
import mppool  # [mppool]
import metrics  # [metrics]
import lazyrci  # [lazy]

# 此代码通过比较以下算法：
# （1）子问题动态规划；（2）子问题动态规划+有效不等式；（3）子问题动态规划+有效不等式+并行计算
# 来检验以下组件的加速效果：
# （1）有效不等式；（2）并行计算；（3）子问题的动态规划求解【该部分需要与配套的子问题gurobi代码求解比较】
def _branch_order(keys):  # [detfix] 分支候选操作的固定次序：服务客户多的在前，同数按 key 字典序
    def n_served(k):
        inside = k[k.index('[') + 1:k.index(']')]
        return len([x for x in inside.split(',') if x.strip()])
    return sorted(set(keys), key=lambda k: (-n_served(k), k))


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



def RHSnum(demand, CustomerLst, subSet, vehicleCap):  # 输入所有客户的需求，name列表，客户集合C’，所使用的卡车索引，各个卡车的容量，输出最少的使用数量
    # 从大到小整理出所使用的车的容量

    # 计算客户集合C’的totalDemand
    totalDemand = 0
    for jname in subSet:  # C'中的客户name
        jindex = CustomerLst.index(jname)
        totalDemand += demand[jindex]
    pass  # [silence] print("totalDemand:", totalDemand)
    # 开始计数
    maxCap = max(vehicleCap)
    cnt = math.ceil(totalDemand/maxCap)
    return cnt


def safe_str(value):
    return str(value) if value is not None else ''


class Subproblem:
    def __init__(self):
        self.customerPi_DICT = {}  # 客户服务约束，对应的乘子，【格式：客户name：乘子】
        self.vehiclePi = 0  # 车辆服务约束，对应的乘子，【格式：数值】
        self.RMPbranchCons_DICT = {}  # 主问题加的分支约束，对应的乘子，【格式：约束name：乘子】
        self.capacityPi_DICT = {}  # 用于加强RLMP的rounded capacity 有效不等式，对应的乘子，【格式：子集索引：乘子】  # 与network.allSubset顺序相对应
        self.noUse_LST = []  # 不能被使用的、操作的名字，【格式：操作key】
        self.mustUse_LST = []  # 必须被使用的、操作的名字，【格式：操作key】
        self.reducedCost = None  # 子问题对应的检验数（考虑了各种乘子之后的）【格式：RC数值】
        self.completionTime = None  # 子问题的完工时间（这一个卡车组的完工时间）
        self.operations = None  # 子问题的所有操作字典


class Node:
    def __init__(self):
        self.RMP = None
        self.SP_lst = []  # 子问题列表，子问题的数量等于车辆数，每辆车都是一个子问题，里面记录着该子问题存储各种乘子的字典，结构是[子问题0对应的字典，子问题1对应的字典]...
        self.X = {}  # SP变量
        self.Y = {}  # SP变量
        self.D = {}  # SP变量
        self.localUB = np.inf
        self.localLB = 0
        self.Lambda = {}  # 主问题RMP的变量的键值对，不是变量字典，是值字典
        self.int_lambda = {}  # 整数变量的键值对
        self.RMPcons = []  # 记录分支过程中，向RMP中所添加的某基系数=0（NOT）或者某基系数=1（YES），所对应的约束名字

        self.noInteger_var_lst = []  # 存放主问题RMP非整数变量
        self.is_integer = False
        self.columnPool = {}  # 主问题的列池,lambda变量名：[基]
        self.columnPool_SP = {}  # 主问题的列池对应的子问题的解，lambda变量名：[子问题中对应的x=1的自变量]
        self.rciSubsets = []  # [lazy] 本节点 RMP 里已加入的 rounded capacity 割（客户子集列表）


    def deepcopy(node):
        new_node = Node()  # 实例化
        new_node.RMP = node.RMP.copy()  # 复制后新结点的模型与复制前相同
        lst = []
        for i in range(0, len(node.SP_lst)):
            lst.append(copy.deepcopy(node.SP_lst[i]))
        new_node.SP_lst = copy.deepcopy(lst)
        new_node.X = copy.deepcopy(node.X.copy())
        new_node.Y = copy.deepcopy(node.Y.copy())
        new_node.D = copy.deepcopy(node.D.copy())
        new_node.localUB = np.inf
        new_node.localLB = node.localLB  # [metrics] 原为 0，导致 globalLB 恒为 0
        new_node.Lambda = copy.deepcopy(node.Lambda)
        new_node.RMPcons = copy.deepcopy(node.RMPcons)

        new_node.int_lambda = copy.deepcopy(node.int_lambda)
        new_node.branch_var_lst = []
        new_node.is_integer = node.is_integer
        new_node.columnPool = copy.deepcopy(node.columnPool)
        new_node.columnPool_SP = copy.deepcopy(node.columnPool_SP)
        new_node.rciSubsets = list(node.rciSubsets)  # [lazy]

        return new_node


def modelingDP(network, iniBase, iniCost):  # 建立初始的受限主问题
    node = Node()

    # 存储不同定价子问题的实例，建立空数据结构
    for _index in range(0, len(network.vehicleLst)):
        SP = Subproblem()
        node.SP_lst.append(SP)

    # 建立初始的受限主问题模型(线性规划问题)
    node.RMP = Model('ini_restrictedMP')
    Lambda = [[] for i in range(0, len(iniBase))]  # 初始的主问题变量lambda
    for i in range(0, len(iniBase)):  # i是第i个子问题
        Lambda[i] = node.RMP.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f'{i}_Lambda_{0}')
        node.columnPool[f'{i}_Lambda_{0}'] = iniBase[i]  # 构建初始列池（这与初始基对应）

    node.columnPool_SP[f'{0}_Lambda_{0}'] = [f'A_{[]}_{0}_{1}_{0}', f'B_{[]}_{1}_{3}_{1}', f'B_{[]}_{3}_{4}_{3}',
                                             f'C_{[25]}_{4}_{4}_{6}', f'B_{[]}_{4}_{3}_{8}', f'B_{[]}_{3}_{12}_{11}',
                                             f'C_{[26]}_{12}_{12}_{15}', f'B_{[]}_{12}_{13}_{17}',
                                             f'C_{[30]}_{13}_{13}_{26}', f'B_{[]}_{13}_{12}_{28}',
                                             f'B_{[]}_{12}_{3}_{37}', f'B_{[]}_{3}_{1}_{41}', f'A_{[]}_{1}_{37}_{43}']

    node.columnPool_SP[f'{1}_Lambda_{0}'] = [f'A_{[]}_{0}_{1}_{0}', f'B_{[]}_{1}_{3}_{1}', f'B_{[]}_{3}_{4}_{3}',
                                             f'B_{[]}_{4}_{11}_{6}', f'C_{[33]}_{11}_{11}_{10}',
                                             f'B_{[]}_{11}_{10}_{12}', f'C_{[34]}_{10}_{10}_{15}',
                                             f'B_{[]}_{10}_{11}_{17}', f'B_{[]}_{11}_{14}_{20}',
                                             f'C_{[27]}_{14}_{14}_{24}', f'B_{[]}_{14}_{15}_{26}',f'C_{[35]}_{15}_{15}_{29}', f'B_{[]}_{15}_{14}_{31}', f'B_{[]}_{14}_{11}_{34}',
                                             f'B_{[]}_{11}_{4}_{38}', f'B_{[]}_{4}_{3}_{42}', f'B_{[]}_{3}_{1}_{45}',
                                             f'A_{[]}_{1}_{37}_{47}']

    node.columnPool_SP[f'{2}_Lambda_{0}'] = [f'A_{[]}_{0}_{1}_{0}', f'B_{[]}_{1}_{2}_{1}', f'C_{[29]}_{2}_{2}_{9}',
                                             f'B_{[]}_{2}_{6}_{11}', f'C_{[31]}_{6}_{6}_{13}', f'B_{[]}_{6}_{8}_{15}',
                                             f'B_{[]}_{8}_{7}_{17}', f'B_{[]}_{7}_{18}_{20}',
                                             f'C_{[28]}_{18}_{18}_{22}', f'B_{[]}_{18}_{16}_{24}',
                                             f'C_{[32]}_{16}_{16}_{27}', f'B_{[]}_{16}_{17}_{29}',
                                             f'B_{[]}_{17}_{19}_{31}', f'C_{[36]}_{19}_{19}_{33}',
                                             f'B_{[]}_{19}_{17}_{35}', f'B_{[]}_{17}_{16}_{37}',
                                             f'B_{[]}_{16}_{8}_{39}', f'B_{[]}_{8}_{6}_{41}', f'B_{[]}_{6}_{2}_{43}',
                                             f'B_{[]}_{2}_{1}_{45}', f'A_{[]}_{1}_{37}_{53}']

    # 目标函数
    # 目标函数
    obj = 0
    for i in range(0, len(iniCost)):
        obj += iniCost[i] * Lambda[i]
    node.RMP.setObjective(obj, sense=GRB.MINIMIZE)
    # 客户服务约束
    for i in range(0, len(network.Customer)):
        expr = 0
        for j in range(0, len(iniBase)):
            expr += iniBase[j][i] * Lambda[j]
        node.RMP.addConstr(expr >= 1, f'C_{i}')
    # 异质单车环路约束(每辆车不同
    for i in range(0, len(network.vehicleLst)):
        node.RMP.addConstr(Lambda[i] <= 1, f'C_{len(network.Customer)+i}')
    #node.RMP.write('RMP_VRPDSTS.lp')

    return node


def modelingDP_CUT(network, iniBase, iniCost):  # 建立初始的受限主问题
    node = Node()
    allSubset = network.allSubset
    CustomerLst = network.Customer
    demand = network.demand
    maxVehicleCap = network.vehicleCap
    network.allSubset = [_S for _S in network.allSubset if RHSnum(demand, CustomerLst, _S, maxVehicleCap) > 1]  # [rci] 右端项为 1 的子集被覆盖约束蕴含，不再加入
    allSubset = network.allSubset  # [rci]
    network.allSubset = []  # [lazy] 割不再预先放进 RMP，改为列生成收敛后按需分离（lazyrci.separate）
    allSubset = network.allSubset  # [lazy]
    vehicleLst = network.vehicleLst
    vehicleNum = len(vehicleLst)
    # 存储不同定价子问题的实例，建立空数据结构
    for _index in range(0, len(network.vehicleLst)):
        SP = Subproblem()
        node.SP_lst.append(SP)

    # 建立初始的受限主问题模型(线性规划问题)
    node.RMP = Model('ini_restrictedMP')
    Lambda = [[] for i in range(0, len(iniBase))]  # 初始的主问题变量lambda
    for i in range(0, len(iniBase)):  # i是第i个子问题
        Lambda[i] = node.RMP.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f'{i}_Lambda_{0}')
        node.columnPool[f'{i}_Lambda_{0}'] = iniBase[i]  # 构建初始列池（这与初始基对应）

    node.columnPool_SP[f'{0}_Lambda_{0}'] = [f'A_{[]}_{0}_{1}_{0}', f'B_{[]}_{1}_{3}_{1}', f'B_{[]}_{3}_{4}_{3}',
                                             f'C_{[25]}_{4}_{4}_{6}', f'B_{[]}_{4}_{3}_{8}', f'B_{[]}_{3}_{12}_{11}',
                                             f'C_{[26]}_{12}_{12}_{15}', f'B_{[]}_{12}_{13}_{17}',
                                             f'C_{[30]}_{13}_{13}_{26}', f'B_{[]}_{13}_{12}_{28}',
                                             f'B_{[]}_{12}_{3}_{37}', f'B_{[]}_{3}_{1}_{41}', f'A_{[]}_{1}_{37}_{43}']

    node.columnPool_SP[f'{1}_Lambda_{0}'] = [f'A_{[]}_{0}_{1}_{0}', f'B_{[]}_{1}_{3}_{1}', f'B_{[]}_{3}_{4}_{3}',
                                             f'B_{[]}_{4}_{11}_{6}', f'C_{[33]}_{11}_{11}_{10}',
                                             f'B_{[]}_{11}_{10}_{12}', f'C_{[34]}_{10}_{10}_{15}',
                                             f'B_{[]}_{10}_{11}_{17}', f'B_{[]}_{11}_{14}_{20}',
                                             f'C_{[27]}_{14}_{14}_{24}', f'B_{[]}_{14}_{15}_{26}',
                                             f'C_{[35]}_{15}_{15}_{29}', f'B_{[]}_{15}_{14}_{31}',
                                             f'B_{[]}_{14}_{11}_{34}',
                                             f'B_{[]}_{11}_{4}_{38}', f'B_{[]}_{4}_{3}_{42}', f'B_{[]}_{3}_{1}_{45}',
                                             f'A_{[]}_{1}_{37}_{47}']

    node.columnPool_SP[f'{2}_Lambda_{0}'] = [f'A_{[]}_{0}_{1}_{0}', f'B_{[]}_{1}_{2}_{1}', f'C_{[29]}_{2}_{2}_{9}',
                                             f'B_{[]}_{2}_{6}_{11}', f'C_{[31]}_{6}_{6}_{13}', f'B_{[]}_{6}_{8}_{15}',
                                             f'B_{[]}_{8}_{7}_{17}', f'B_{[]}_{7}_{18}_{20}',
                                             f'C_{[28]}_{18}_{18}_{22}', f'B_{[]}_{18}_{16}_{24}',
                                             f'C_{[32]}_{16}_{16}_{27}', f'B_{[]}_{16}_{17}_{29}',
                                             f'B_{[]}_{17}_{19}_{31}', f'C_{[36]}_{19}_{19}_{33}',
                                             f'B_{[]}_{19}_{17}_{35}', f'B_{[]}_{17}_{16}_{37}',
                                             f'B_{[]}_{16}_{8}_{39}', f'B_{[]}_{8}_{6}_{41}', f'B_{[]}_{6}_{2}_{43}',
                                             f'B_{[]}_{2}_{1}_{45}', f'A_{[]}_{1}_{37}_{53}']
    # 目标函数
    # 目标函数
    obj = 0
    for i in range(0, len(iniCost)):
        obj += iniCost[i] * Lambda[i]
    node.RMP.setObjective(obj, sense=GRB.MINIMIZE)
    # 客户服务约束
    for i in range(0, len(network.Customer)):
        expr = 0
        for j in range(0, len(iniBase)):
            expr += iniBase[j][i] * Lambda[j]
        node.RMP.addConstr(expr >= 1, f'C_{i}')
    # 异质单车环路约束(每辆车不同
    for i in range(0, len(network.vehicleLst)):
        node.RMP.addConstr(Lambda[i] <= 1, f'C_{len(network.Customer)+i}')

    # 【加强割】rounded capacity inequalities 先根据初始基，把框架搭起来，然后列生成向里面添加列
    for i in range(0, len(allSubset)):  # i是子集的索引
        expr = 0
        subSet = allSubset[i]  # 客户集合的子集 [10], [11], [12], [13], [10, 11]...
        name_indexLst = []
        for name in subSet:
            name_index = CustomerLst.index(name)  # 确定客户对应的索引
            name_indexLst.append(name_index)  # 如客户子集的索引列表[0,1]
        # print("此时客户子集索引列表是",name_indexLst)

        for vehicle_index in range(0, vehicleNum):  # 两个子问题
            # 分别找出两个子问题下面的初始列里面，哪些经过了客户子集C'中的客户，但凡经过一个，就将初始列加到约束中
            base = iniBase[vehicle_index][:len(network.Customer)]  # 当前子问题对应的列
            flag = 0
            for customerIndex in name_indexLst:  # 当前客户子集C’对应的客户索引
                if base[customerIndex] == 1:  # 说明此时的base经过了客户子集C’中索引为customerIndex的客户
                    flag = 1
            if flag == 1:
                expr += Lambda[vehicle_index]  # 当前车辆的基（路径）iniBase = [[1, 0, 0, 1, 1, 0], [0, 1, 1, 0, 0, 1]]
            # print("当前客户子集下，添加的变量为",f'{vehicle_index}_Lambda_{0}')
        RHS = RHSnum(demand, CustomerLst, subSet, maxVehicleCap)  # 约束右端项
        # print("当前客户子集下，添加的RHS为", RHS)
        # print(f"最终在{subSet}的客户名字下，添加的约束是{expr >= RHS}")
        node.RMP.addConstr(expr >= RHS, f'capacityCons_{subSet}')

    node.RMP.write('Check.lp')
    return node


def modelingDP_CUT_PA(network, iniBase, iniCost):  # 建立初始的受限主问题
    node = Node()
    allSubset = network.allSubset
    CustomerLst = network.Customer
    demand = network.demand
    maxVehicleCap = network.vehicleCap
    network.allSubset = [_S for _S in network.allSubset if RHSnum(demand, CustomerLst, _S, maxVehicleCap) > 1]  # [rci] 右端项为 1 的子集被覆盖约束蕴含，不再加入
    allSubset = network.allSubset  # [rci]
    network.allSubset = []  # [lazy] 割不再预先放进 RMP，改为列生成收敛后按需分离（lazyrci.separate）
    allSubset = network.allSubset  # [lazy]
    vehicleLst = network.vehicleLst
    vehicleNum = len(vehicleLst)
    # 存储不同定价子问题的实例，建立空数据结构
    for _index in range(0, len(network.vehicleLst)):
        SP = Subproblem()
        node.SP_lst.append(SP)

    # 建立初始的受限主问题模型(线性规划问题)
    node.RMP = Model('ini_restrictedMP')
    Lambda = [[] for i in range(0, len(iniBase))]  # 初始的主问题变量lambda
    for i in range(0, len(iniBase)):  # i是第i个子问题
        Lambda[i] = node.RMP.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f'{i}_Lambda_{0}')
        node.columnPool[f'{i}_Lambda_{0}'] = iniBase[i]  # 构建初始列池（这与初始基对应）

    node.columnPool_SP[f'{0}_Lambda_{0}'] = [f'A_{[]}_{0}_{1}_{0}', f'B_{[]}_{1}_{3}_{1}', f'B_{[]}_{3}_{4}_{3}',
                                             f'C_{[25]}_{4}_{4}_{6}', f'B_{[]}_{4}_{3}_{8}', f'B_{[]}_{3}_{12}_{11}',
                                             f'C_{[26]}_{12}_{12}_{15}', f'B_{[]}_{12}_{13}_{17}',
                                             f'C_{[30]}_{13}_{13}_{26}', f'B_{[]}_{13}_{12}_{28}',
                                             f'B_{[]}_{12}_{3}_{37}', f'B_{[]}_{3}_{1}_{41}', f'A_{[]}_{1}_{37}_{43}']

    node.columnPool_SP[f'{1}_Lambda_{0}'] = [f'A_{[]}_{0}_{1}_{0}', f'B_{[]}_{1}_{3}_{1}', f'B_{[]}_{3}_{4}_{3}',
                                             f'B_{[]}_{4}_{11}_{6}', f'C_{[33]}_{11}_{11}_{10}',
                                             f'B_{[]}_{11}_{10}_{12}', f'C_{[34]}_{10}_{10}_{15}',
                                             f'B_{[]}_{10}_{11}_{17}', f'B_{[]}_{11}_{14}_{20}',
                                             f'C_{[27]}_{14}_{14}_{24}', f'B_{[]}_{14}_{15}_{26}',
                                             f'C_{[35]}_{15}_{15}_{29}', f'B_{[]}_{15}_{14}_{31}',
                                             f'B_{[]}_{14}_{11}_{34}',
                                             f'B_{[]}_{11}_{4}_{38}', f'B_{[]}_{4}_{3}_{42}', f'B_{[]}_{3}_{1}_{45}',
                                             f'A_{[]}_{1}_{37}_{47}']

    node.columnPool_SP[f'{2}_Lambda_{0}'] = [f'A_{[]}_{0}_{1}_{0}', f'B_{[]}_{1}_{2}_{1}', f'C_{[29]}_{2}_{2}_{9}',
                                             f'B_{[]}_{2}_{6}_{11}', f'C_{[31]}_{6}_{6}_{13}', f'B_{[]}_{6}_{8}_{15}',
                                             f'B_{[]}_{8}_{7}_{17}', f'B_{[]}_{7}_{18}_{20}',
                                             f'C_{[28]}_{18}_{18}_{22}', f'B_{[]}_{18}_{16}_{24}',
                                             f'C_{[32]}_{16}_{16}_{27}', f'B_{[]}_{16}_{17}_{29}',
                                             f'B_{[]}_{17}_{19}_{31}', f'C_{[36]}_{19}_{19}_{33}',
                                             f'B_{[]}_{19}_{17}_{35}', f'B_{[]}_{17}_{16}_{37}',
                                             f'B_{[]}_{16}_{8}_{39}', f'B_{[]}_{8}_{6}_{41}', f'B_{[]}_{6}_{2}_{43}',
                                             f'B_{[]}_{2}_{1}_{45}', f'A_{[]}_{1}_{37}_{53}']
    # 目标函数
    obj = 0
    for i in range(0, len(iniCost)):
        obj += iniCost[i] * Lambda[i]
    node.RMP.setObjective(obj, sense=GRB.MINIMIZE)
    # 客户服务约束
    for i in range(0, len(network.Customer)):
        expr = 0
        for j in range(0, len(iniBase)):
            expr += iniBase[j][i] * Lambda[j]
        node.RMP.addConstr(expr >= 1, f'C_{i}')
    # 异质单车环路约束(每辆车不同
    for i in range(0, len(network.vehicleLst)):
        node.RMP.addConstr(Lambda[i] <= 1, f'C_{len(network.Customer)+i}')

    # 【加强割】rounded capacity inequalities 先根据初始基，把框架搭起来，然后列生成向里面添加列
    for i in range(0, len(allSubset)):  # i是子集的索引
        expr = 0
        subSet = allSubset[i]  # 客户集合的子集 [10], [11], [12], [13], [10, 11]...
        name_indexLst = []
        for name in subSet:
            name_index = CustomerLst.index(name)  # 确定客户对应的索引
            name_indexLst.append(name_index)  # 如客户子集的索引列表[0,1]
        # print("此时客户子集索引列表是",name_indexLst)

        for vehicle_index in range(0, vehicleNum):  # 两个子问题
            # 分别找出两个子问题下面的初始列里面，哪些经过了客户子集C'中的客户，但凡经过一个，就将初始列加到约束中
            base = iniBase[vehicle_index][:len(network.Customer)]  # 当前子问题对应的列
            flag = 0
            for customerIndex in name_indexLst:  # 当前客户子集C’对应的客户索引
                if base[customerIndex] == 1:  # 说明此时的base经过了客户子集C’中索引为customerIndex的客户
                    flag = 1
            if flag == 1:
                expr += Lambda[vehicle_index]  # 当前车辆的基（路径）iniBase = [[1, 0, 0, 1, 1, 0], [0, 1, 1, 0, 0, 1]]
            # print("当前客户子集下，添加的变量为",f'{vehicle_index}_Lambda_{0}')
        RHS = RHSnum(demand, CustomerLst, subSet, maxVehicleCap)  # 约束右端项
        # print("当前客户子集下，添加的RHS为", RHS)
        pass  # [silence] print(f"最终在{subSet}的客户名字下，添加的约束是{expr >= RHS}")
        node.RMP.addConstr(expr >= RHS, f'capacityCons_{subSet}')

    node.RMP.write('Check.lp')
    return node


def columnGenerationDP(node, network,instanceName):  # 输出完备的RMP
    node.RMP.setParam("OutputFlag", 0)
    node.RMP.optimize()
    adjustCoff = 100000000  # 用于让最短路经过某操作的调整值
    # 进入CG循环
    SPtotalTimeCost = 0  # 子问题计算用时
    RMPtotalTimeCost = 0  # 主问题计算用时
    Iter = 0
    if node.RMP.Status != GRB.OPTIMAL:  # 由于行的加入，主问题可能无解
        pass  # [silence] print("——————该RMP不可行，割掉当前节点——————")
        return node,SPtotalTimeCost, RMPtotalTimeCost

    pass  # [silence] print("________________________↓开始列生成↓_______________________")
    while (node.RMP.Status == GRB.OPTIMAL):
        if metrics.expired(): break  # [wall] 某个节点的列生成收敛不了时，外层判据永远轮不到
        metrics.REC.d['cg_iters'] += 1  # [metrics]
        for var in node.RMP.getVars():  # 对于RMP解中的任意一个变量
            node.Lambda[var.VarName] = var.x
        pass  # [silence] print('在本次列生成前，主问题变量node.Lambda有:', node.Lambda)
        sumRC = 0  # 初始检验数之和（求解若干子问题）
        for _index in range(0, len(node.SP_lst)):  # 几辆车就有几个子问题，对于这一个子问题而言，需要计算最短路

            SP_i = node.SP_lst[_index]  # 子问题实例
            # 根据主问题变量数量，结合当前子问题情况，来设置生成列的命名起点：（1）先找到属于这一个子问题的列
            prefix = f"{_index}"
            filtered_keys = [key for key in node.Lambda.keys() if key.startswith(prefix)]  # 筛选出这个子问题i下面的所有相关的lambda
            # print('filtered_keys',filtered_keys)
            if len(filtered_keys) == 0:
                Iter = 0
            else:
                a = filtered_keys[-1].split('_')
                if a[1] != 'newLambda':
                    Iter = 0
                elif a[1] == 'newLambda':
                    Iter = int(a[2])

            # 填补此时子问题实例所需要的客户服务乘子 SP_i.customerPi_DICT
            for custIndex in range(0, len(network.Customer)):
                customer_name = network.Customer[custIndex]
                multiper_j = node.RMP.getConstrByName(f'C_{custIndex}').pi
                SP_i.customerPi_DICT[f"{customer_name}"] = multiper_j
            pass  # [silence] print(f"此时子问题{_index}的实例所需要的客户服务乘子:{SP_i.customerPi_DICT}")

            # 填补此时子问题实例所需要的车辆服务乘子 SP_i.vehiclePi
            SP_i.vehiclePi = node.RMP.getConstrByName(f'C_{len(network.Customer) + _index}').pi
            pass  # [silence] print(f"此时子问题{_index}的实例所需要的车辆服务乘子:{SP_i.vehiclePi}")

            # 填补此时子问题实例所需要的分支约束对应的乘子  SP_i.RMPbranchCons_DICT，将对应的操作RC成本加上乘子即可
            for consName in node.RMPcons:  # 这个是在branch过程中向主问题添加的分支约束 f'NOT_{SP_index}_' + A_[]_i_j_operation.startTime
                op = consName.split('_')
                if int(op[1]) == _index:  # 是当前子问题
                    multiper = node.RMP.getConstrByName(consName).pi  # 这条约束的乘子
                    opKey = f'{op[2]}_{op[3]}_{op[4]}_{op[5]}_{op[6]}'  # 该操作的key值
                    SP_i.RMPbranchCons_DICT[opKey] = multiper
            pass  # [silence] print(f"此时子问题{_index}的实例所需要分支约束对应的乘子:{SP_i.RMPbranchCons_DICT}")

            SP_i.reducedCost = 0  # 这一个子问题对应的检验数（=考虑客户乘子的最短路成本+车辆约束乘子+分支约束乘子+后续加强RMP乘子）

            if SP_i.operations is None:  # 若第一个子问题还未构建操作，那就构建
                if _index == 0:  # 是第一个子问题
                    # 定义文件路径
                    file_dir = fr'{RESULT_DIR}/Performance of acceleration components/{instanceName}'
                    file_path = os.path.join(file_dir, 'iniOperation.txt')
                    try:
                        # 尝试读取文件
                        OperationDict = {}
                        operatiob_df = pd.read_csv(file_path, sep="\|", engine='python')

                        for row in range(0, len(operatiob_df)):
                            operation = Operation()
                            operation.startID = int(operatiob_df.loc[row, 'startID'])  # 操作起点
                            operation.startTime = int(operatiob_df.loc[row, 'startTime'])  # 操作开始时间
                            operation.endID = int(operatiob_df.loc[row, 'endID'])  # 操作终点
                            operation.endTime = int(operatiob_df.loc[row, 'endTime'])  # 操作结束时间
                            operation.VehicleId = None
                            operation.operationName = operatiob_df.loc[row, 'operationName']
                            operation.key = operatiob_df.loc[row, 'key']
                            operation.same = operatiob_df.loc[row, 'same']  # 肯定是同一个
                            operation.droneTimeCost = int(operatiob_df.loc[row, 'droneTimeCost'])
                            operation.truckTimeCost = int(operatiob_df.loc[row, 'truckTimeCost'])
                            operation.operationTimeCost = int(operatiob_df.loc[row, 'operationTimeCost'])
                            operation.droneServiceID = []  # 出车操作无客户服务
                            operation.truckServiceID = []  # 出车操作无客户服务
                            operation.ServiceID = []  # 该操作无客户服务

                            if str(operatiob_df.loc[row, 'droneServiceID']) == 'nan':
                                pass
                            else:
                                droneServiceLst = str(operatiob_df.loc[row, 'droneServiceID']).split('_')
                                for idx in range(0, len(droneServiceLst) - 1):
                                    operation.droneServiceID.append(int(droneServiceLst[idx]))

                            if str(operatiob_df.loc[row, 'truckServiceID']) == 'nan':
                                pass
                            else:
                                truckServiceLst = str(operatiob_df.loc[row, 'truckServiceID']).split('_')
                                for idx in range(0, len(truckServiceLst) - 1):
                                    operation.truckServiceID.append(int(truckServiceLst[idx]))

                            if str(operatiob_df.loc[row, 'ServiceID']) == 'nan':
                                pass
                            else:
                                ServiceLst = str(operatiob_df.loc[row, 'ServiceID']).split('_')
                                for idx in range(0, len(ServiceLst) - 1):
                                    operation.ServiceID.append(int(ServiceLst[idx]))
                            operation.ServiceDemand = int(operatiob_df.loc[row, 'ServiceDemand'])
                            operation.sumOfCustomerPi = int(operatiob_df.loc[row, 'sumOfCustomerPi'])
                            operation.ReducedCost = int(operatiob_df.loc[row, 'ReducedCost'])
                            OperationDict[operation.key] = operation
                        SP_i.operations = OperationDict

                    except FileNotFoundError:
                        # 处理文件不存在的情况
                        pass  # [silence] print(f"警告: 文件 {file_path} 不存在，尝试动态生成操作数据...")
                        # 1. 确保目录存在
                        os.makedirs(file_dir, exist_ok=True)  # 自动创建目录
                        # 2. 生成默认操作数据

                        # 3.构建Operation
                        SP_i.operations = cO.FEoperationFunction(network, network.Truckcapacity[_index],network.UAVcapacity[_index], network.UAVendurance[_index])

                        # 2.调整op.ReducedCost
                        for op in SP_i.operations.values():
                            # 看这一个操作经过了哪些客户，然后把对应约束的乘子求和，最后放到这个操作上去
                            piSum1 = 0
                            for custerName in op.ServiceID:
                                piSum1 += SP_i.customerPi_DICT[f'{custerName}']
                            op.sumOfCustomerPi = piSum1
                            op.ReducedCost = op.operationTimeCost - op.sumOfCustomerPi

                        # 将操作进行保存
                        result_File = open(file_path, 'w').close()
                        result_generate = open(file_path, mode='a')
                        result_generate.write(str('startID') + '|' + str('startTime') + '|' + str('endID') + '|' + str(
                                    'endTime') + '|' + str('VehicleId') + '|' + str('operationName') + '|' + str('key') + '|' + str(
                                    'same') + '|' + str('droneTimeCost') + '|' + str('truckTimeCost') + '|' + str(
                                    'operationTimeCost') + '|' + str('droneServiceID') + '|' + str(
                                    'truckServiceID') + '|' + str('ServiceID') + '|' + str('ServiceDemand') + '|' + str(
                                    'sumOfCustomerPi') + '|' + str(
                                    'ReducedCost'))
                        result_generate.write('\n')
                        for op in SP_i.operations.values():
                            droneServiceStr = ''
                            truckServiceStr = ''
                            ServiceStr = ''
                            if op.droneServiceID == []:
                                pass
                            else:
                                for _id in op.droneServiceID:
                                    droneServiceStr = droneServiceStr + str(f'{_id}_')

                            if op.truckServiceID == []:
                                pass
                            else:
                                for _id in op.truckServiceID:
                                    truckServiceStr = truckServiceStr + str(f'{_id}_')

                            if op.ServiceID == []:
                                pass
                            else:
                                for _id in op.ServiceID:
                                    ServiceStr = ServiceStr + str(f'{_id}_')

                            result_generate.write(
                                safe_str(op.startID) + '|' +
                                safe_str(op.startTime) + '|' +
                                safe_str(op.endID) + '|' +
                                safe_str(op.endTime) + '|' +
                                safe_str(op.VehicleId) + '|' +
                                safe_str(op.operationName) + '|' +
                                safe_str(op.key) + '|' +
                                safe_str(op.same) + '|' +
                                safe_str(op.droneTimeCost) + '|' +
                                safe_str(op.truckTimeCost) + '|' +
                                safe_str(op.operationTimeCost) + '|' +
                                safe_str(droneServiceStr) + '|' +
                                safe_str(truckServiceStr) + '|' +
                                safe_str(ServiceStr) + '|' +
                                safe_str(op.ServiceDemand) + '|' +
                                safe_str(op.sumOfCustomerPi) + '|' +
                                safe_str(op.ReducedCost)
                            )
                            result_generate.write('\n')
                        result_generate.close()
                else:
                    SP_i.operations = copy.deepcopy(node.SP_lst[0].operations)
                    # 复制
            # [dualfix] 不论操作是本轮新建还是已存在，都用当前对偶重算检验数
            # 只需要调整乘子
            for op in SP_i.operations.values():
                piSum = 0
                for custerName in op.ServiceID:
                    piSum += SP_i.customerPi_DICT[f'{custerName}']
                op.sumOfCustomerPi = piSum
                op.ReducedCost = op.operationTimeCost - op.sumOfCustomerPi

            # 下面是设计到分支的一些信息，是多个节点之间的更新，根据self.noUse_LST、 self.mustUse_LST删除Operations里面的一些操作，以及调整对应的检验数
            pass  # [silence] print(f"此时子问题{_index}的分支约束键",SP_i.RMPbranchCons_DICT.keys())

            # (一)、根据禁止使用的操作key值，在当前操作字典中找到对应操作，并删除
            for del_op in SP_i.noUse_LST:  # 对于需要删除的操作：del_op
                if del_op in SP_i.operations:
                    del SP_i.operations[del_op]
                    pass  # [silence] print("删除这个操作(不能经过)：", del_op)
                    # SP_i.noUse_LST.remove(del_op)  # 既然已经在.operations中删除了，那就在.noUse_LST中删除，其实就相当于表示已经完成了工作
                else:
                    pass
                    # print(f"需要删除的操作{del_op}竟然不在列表中，需要检查")

            # (二)、根据主问题加的分支约束，来修改对应操作的operation.ReducedCost【存疑】
            # 仔细思考：对于分支约束的禁止使用的操作，其已经被删除了，不可能出现在下次的列中，也就无需对其进行调整
            # 而对于分支约束中必须使用的操作，尽管其一定会出现在下一次的列生成中，应该是需要对其增加一个branchMultiper，保证列检验数计算的完整性
            for branchOperationKey in SP_i.RMPbranchCons_DICT.keys():
                branchMultiper = SP_i.RMPbranchCons_DICT[branchOperationKey]
                if branchOperationKey in SP_i.operations.keys():  # 如果是不能经过，那么已经删除了，没必要调整
                    op = SP_i.operations[branchOperationKey]
                    pass  # [silence] print("分支导致的该操作检验数的变化，(具体而言是必须访问)的那个操作为：", op.key)
                    op.sumOfCustomerPi = op.sumOfCustomerPi + branchMultiper
                    op.ReducedCost -= branchMultiper  # 因为op.ReducedCost = op.operationTimeCost - op.sumOfCustomerPi
                else:
                    pass

            # (三)、根据必须使用的操作key值，在当前操作字典中找到对应操作，利用大数adjustCoff调整其reduceCost，以保证后续一定会使用
            for adjust_op in SP_i.mustUse_LST:  # 对于需要调整的操作：adjust_op
                if adjust_op in SP_i.operations:
                    SP_i.operations[adjust_op].sumOfCustomerPi = SP_i.operations[adjust_op].sumOfCustomerPi + adjustCoff
                    SP_i.operations[adjust_op].ReducedCost -= adjustCoff  # 因为op.ReducedCost = op.operationTimeCost - op.sumOfCustomerPi
                    pass  # [silence] print("利用大数adjustCoff调整这个操作，使其后面最短路中必须经过，这个操作为：", adjust_op)
                else:
                    pass  # [silence] print(f"需要调整的操作{adjust_op}竟然不在列表中，需要检查")

            # 当调整完之后，就可以计算最短路了，只不过还需要对最短路的结果进行一下修正（有调整值的加入，需要去除调整值）
            SPstartTime = time.perf_counter()  # 子问题求解开始时间
            result = dp_SPP.shortest_path_operations(SP_i.operations, network, network.vehicleCap[_index])
            SPendTime = time.perf_counter()  # 子问题求解结束时间
            SPtotalTimeCost += SPendTime - SPstartTime
            metrics.REC.add_pricing(SPendTime - SPstartTime)  # [metrics]

            if result is None:
                raise ("问题有误")
            else:
                op = []
                pass  # [silence] print(f"计算得到(考虑客户乘子、分支约束乘子)的最短路成本为：{result[0]}")
                for opUsedIndex in range(0, len(result[1])):  # 当前生成的路径的所有操作集合
                    opUsedKey = result[1][opUsedIndex].key
                    op.append(opUsedKey)
                pass  # [silence] print(f"计算得到的最短路所选择的操作为：{op}")

                SP_i.completionTime = result[1][-1].endTime
                SP_i.reducedCost = result[0] - SP_i.vehiclePi  # 减去车辆服务乘子，这样的话就是考虑了客户服务、车辆服务、分支的乘子后的RC了
                # 对最短路成本 SP_i.reducedCost 进行回调
                for opUsedIndex in range(0, len(result[1])):  # 当前生成的路径的所有操作集合
                    opUsedKey = result[1][opUsedIndex].key
                    if opUsedKey in SP_i.mustUse_LST:
                        SP_i.reducedCost = SP_i.reducedCost + adjustCoff  # 有调整值的加入，需要去除调整值
                pass  # [silence] print(f"回调后当前子问题SP_i的reducedCost为：{SP_i.reducedCost}")
                # 得到具体的哪些客户被什么（卡车、无人机、或者两者共同）来访问了
                truckAlone = []
                droneAlone = []
                together = []
                for i in range(0, len(result[1])):
                    operation = result[1][i]
                    # print(operation.operationName,f"时间从{operation.startTime}到{operation.endTime}，位置从{operation.startID}到{operation.endID}，RC成本是{operation.ReducedCost}")
                    if operation.same == True and len(operation.ServiceID) > 0:  # 说明有客户被卡车和无人机一起访问了(只有C类操作)，添加进together列表
                        together += operation.ServiceID
                    if operation.same == False:  # D类操作或者E类操作
                        truckAlone += operation.truckServiceID
                        droneAlone += operation.droneServiceID
                pass  # [silence] print(f"在这个生成的列（路径）中，卡车访问的客户为：{truckAlone},无人机访问的客户为：{droneAlone},共同访问的客户为：{together}")
                allCustomer = truckAlone+droneAlone+together
                pass  # [silence] print(f"所服务的客户为{allCustomer}")

                sumRC += round(SP_i.reducedCost, 4)

            if _index == len(node.SP_lst) - 1 and sumRC >= 0:  # 对于最后一个子问题，如果到现在的sum都是0（说明这个最后一个子问题的值也是0），则1.不需要进行列生成，跳出 for 循环（最后一个位置continue也可以实现），2.跳出while循环
                break

            if round(SP_i.reducedCost, 4) >= 0:  # 任何一个检验数大于等于0的子问题，都不需要进行列生成，而是直接跳到下一个
                continue

            Iter += 1
            pass  # [silence] print(f"在主问题完备前，当前列生成的迭代次数：{Iter}，RMP目标函数值为：{node.RMP.ObjVal}，当前子问题{_index}目标函数值（该列的成本）为：{SP_i.reducedCost}")
            pass  # [silence] print("构建列对象并将新列加入RMP")
            # 构建列对象并将新列加入RMP
            col = Column()  # 新建列对象，为了有效地进行列管理
            col_lst = []  # 存放列的一些信息，展示用
            col_lst_SP = []  # 存放某个列经过了哪些操作，这里记录了操作的key值，会用在分支阶段中

            # 生成列
            # (一)、客户服务约束对应的列
            for custIndex in range(0, len(network.Customer)):  # 记录每一个客户都是怎么服务的
                customer_name = network.Customer[custIndex]
                alpha_i = 0
                beta_i = 0
                gama_i = 0
                if customer_name not in allCustomer:  # 该客户没有被这个路径（列）服务
                    coeff = alpha_i + beta_i + gama_i
                    col_lst.append((alpha_i, beta_i, gama_i))
                    col.addTerms(coeff, node.RMP.getConstrByName(f'C_{custIndex}'))
                else:  # 该客户被这个路径（列）服务了
                    if customer_name in truckAlone: # 被卡车单独访问
                        alpha_i = 1
                    if customer_name in droneAlone: # 被无人机单独访问
                        beta_i = 1
                    if customer_name in together: # 被卡车和无人机共同访问
                        gama_i = 1
                    coeff = alpha_i + beta_i + gama_i
                    col_lst.append((alpha_i, beta_i, gama_i))
                    col.addTerms(coeff, node.RMP.getConstrByName(f'C_{custIndex}'))

            # (二)、车辆使用约束对应的列
            for i in range(0,len(network.vehicleLst)):  # 车辆路径数量约束
                if i == _index:  # 如果对应的约束就是应当施加在这个子问题（车辆）上面的
                    col.addTerms(1, node.RMP.getConstrByName(f'C_{len(network.Customer)+i}'))  # 异质车辆路径数量约束
                    col_lst.append(1)
                '''else:
                    col.addTerms(0, node.RMP.getConstrByName(f'C_{len(network.Customer)+i}'))  # 异质车辆路径数量约束
                    col_lst.append(0)'''

            # (三)、分支约束对应的列
            for i in range(0, len(node.RMPcons)):  # 分支约束  f'NOT_{SP_index}_' + A_[]_i_j_operation.startTime
                op = node.RMPcons[i].split('_')  # 约束名字
                if int(op[1]) == _index:  # 是当前子问题生成的列
                    opKey = f'{op[2]}_{op[3]}_{op[4]}_{op[5]}_{op[6]}'  # 这个分支约束所对应的操作key值
                    # 判断生成的这个路径是否有使用这个操作opName，如果有的话，当前列的该位置就是1，没有的话就是0
                    a_i = 0
                    for opUsedIndex in range(0,len(result[1])):  # 当前生成的路径的所有操作集合
                        opUsedKey = result[1][opUsedIndex].key
                        if opUsedKey == opKey:
                            a_i = 1
                            break
                    col.addTerms(a_i, node.RMP.getConstrByName(node.RMPcons[i]))
                    col_lst.append(a_i)

            # 生成这个列对应的操作key，用col_lst_SP存储
            for opUsedIndex in range(0, len(result[1])):  # 当前生成的路径的所有操作集合
                opUsedKey = result[1][opUsedIndex].key
                col_lst_SP.append(opUsedKey)

            pass  # [silence] print("生成的列为：", col_lst)
            pass  # [silence] print('生成列的操作详情为：', col_lst_SP)

            cp = SP_i.completionTime  # 此列对应的路由成本（花费的时间）

            node.RMP.addVar(obj=cp, column=col, lb=0, ub=1, name=f"{_index}_newLambda_" + str(Iter))
            node.columnPool[f"{_index}_newLambda_" + str(Iter)] = col_lst  # 随着列生成不断丰富列池
            node.columnPool_SP[f"{_index}_newLambda_" + str(Iter)] = col_lst_SP  # 随着列生成不断丰富列池

        pass  # [silence] print(f'所有子问题RC之和为{sumRC}，相当于表示目前的总Gap为：{-sumRC}')
        if sumRC >= 0:  # 子问题的检验数之和等于0
            break

        node.RMP.update()
        RMPstartTime = time.perf_counter()  # 主问题求解开始时间
        node.RMP.optimize()
        RMPendTime = time.perf_counter()  # 主问题求解结束时间
        RMPtotalTimeCost += RMPendTime - RMPstartTime

    for var in node.RMP.getVars():  # 对于RMP解中的任意一个变量
        node.Lambda[var.VarName] = var.x
    pass  # [silence] print('在列生成后，主问题变量node.Lambda有:',node.Lambda)
    pass  # [silence] print("________________________↑列生成结束↑_______________________")
    pass  # [silence] print()
    # node.RMP.write(f"final_RMP_VRPDSTS.lp")

    return node,SPtotalTimeCost,RMPtotalTimeCost


def columnGenerationDP_CUT(node, network,instanceName):  # 输出完备的RMP
    node.RMP.setParam("OutputFlag", 0)
    network.allSubset = node.rciSubsets  # [lazy] 本节点已加入的割
    node.RMP.optimize()
    adjustCoff = 100000000  # 用于让最短路经过某操作的调整值
    # 进入CG循环
    SPtotalTimeCost = 0  # 子问题计算用时
    RMPtotalTimeCost = 0  # 主问题计算用时
    Iter = 0
    if node.RMP.Status != GRB.OPTIMAL:  # 由于行的加入，主问题可能无解
        pass  # [silence] print("——————该RMP不可行，割掉当前节点——————")
        return node,SPtotalTimeCost, RMPtotalTimeCost

    pass  # [silence] print("________________________↓开始列生成↓_______________________")
    while (node.RMP.Status == GRB.OPTIMAL):
        if metrics.expired(): break  # [wall] 某个节点的列生成收敛不了时，外层判据永远轮不到
        metrics.REC.d['cg_iters'] += 1  # [metrics]
        for var in node.RMP.getVars():  # 对于RMP解中的任意一个变量
            node.Lambda[var.VarName] = var.x
        pass  # [silence] print('在本次列生成前，主问题变量node.Lambda有:', node.Lambda)
        sumRC = 0  # 初始检验数之和（求解若干子问题）
        for _index in range(0, len(node.SP_lst)):  # 几辆车就有几个子问题，对于这一个子问题而言，需要计算最短路

            SP_i = node.SP_lst[_index]  # 子问题实例
            # 根据主问题变量数量，结合当前子问题情况，来设置生成列的命名起点：（1）先找到属于这一个子问题的列
            prefix = f"{_index}"
            filtered_keys = [key for key in node.Lambda.keys() if key.startswith(prefix)]  # 筛选出这个子问题i下面的所有相关的lambda
            # print('filtered_keys',filtered_keys)
            if len(filtered_keys) == 0:
                Iter = 0
            else:
                a = filtered_keys[-1].split('_')
                if a[1] != 'newLambda':
                    Iter = 0
                elif a[1] == 'newLambda':
                    Iter = int(a[2])

            # 填补此时子问题实例所需要的客户服务乘子 SP_i.customerPi_DICT
            for custIndex in range(0, len(network.Customer)):
                customer_name = network.Customer[custIndex]
                multiper_j = node.RMP.getConstrByName(f'C_{custIndex}').pi
                SP_i.customerPi_DICT[f"{customer_name}"] = multiper_j
            pass  # [silence] print(f"此时子问题{_index}的实例所需要的客户服务乘子:{SP_i.customerPi_DICT}")

            # 填补此时子问题实例所需要的车辆服务乘子 SP_i.vehiclePi
            SP_i.vehiclePi = node.RMP.getConstrByName(f'C_{len(network.Customer) + _index}').pi
            pass  # [silence] print(f"此时子问题{_index}的实例所需要的车辆服务乘子:{SP_i.vehiclePi}")

            # 填补此时子问题实例所需要的分支约束对应的乘子  SP_i.RMPbranchCons_DICT，将对应的操作RC成本加上乘子即可
            for consName in node.RMPcons:  # 这个是在branch过程中向主问题添加的分支约束 f'NOT_{SP_index}_' + A_[]_i_j_operation.startTime
                op = consName.split('_')
                if int(op[1]) == _index:  # 是当前子问题
                    multiper = node.RMP.getConstrByName(consName).pi  # 这条约束的乘子
                    opKey = f'{op[2]}_{op[3]}_{op[4]}_{op[5]}_{op[6]}'  # 该操作的key值
                    SP_i.RMPbranchCons_DICT[opKey] = multiper
            pass  # [silence] print(f"此时子问题{_index}的实例所需要分支约束对应的乘子:{SP_i.RMPbranchCons_DICT}")

            # 填补此时子问题实例所需要的rounded capacity inequalities对应的乘子  SP_i.capacityPi_DICT
            SP_i.capacityPi_DICT = {}  # [lazy] 只保留本节点当前已加入的割的乘子
            for custSubIndex in range(0, len(network.allSubset)):  # 所有的子集都对应有一个约束
                custerSubNameSet = network.allSubset[custSubIndex]
                multiper_B = node.RMP.getConstrByName(f'capacityCons_{custerSubNameSet}').pi
                SP_i.capacityPi_DICT[f"{custerSubNameSet}"] = multiper_B
            pass  # [silence] print(f"此时子问题{_index}的实例所需要的rounded capacity inequalities对应的乘子:{SP_i.capacityPi_DICT}")

            SP_i.reducedCost = 0  # 这一个子问题对应的检验数（=考虑客户乘子的最短路成本+车辆约束乘子+分支约束乘子+后续加强RMP乘子）
            if SP_i.operations == None:  # 若第一个子问题还未构建操作，那就构建
                if _index == 0:  # 是第一个子问题
                    OperationDict = {}
                    # 1.调取Operation并存储
                    operatiob_df = pd.read_csv(fr'{RESULT_DIR}/Performance of acceleration components/{instanceName}/iniOperation.txt',sep="\|",engine='python')
                    for row in range(0, len(operatiob_df)):
                        operation = Operation()
                        operation.startID = int(operatiob_df.loc[row,'startID'])  # 操作起点
                        operation.startTime = int(operatiob_df.loc[row,'startTime'])  # 操作开始时间
                        operation.endID = int(operatiob_df.loc[row,'endID'])  # 操作终点
                        operation.endTime = int(operatiob_df.loc[row,'endTime'])  # 操作结束时间
                        operation.VehicleId = None
                        operation.operationName = operatiob_df.loc[row,'operationName']
                        operation.key = operatiob_df.loc[row,'key']
                        operation.same = operatiob_df.loc[row,'same']  # 肯定是同一个
                        operation.droneTimeCost = int(operatiob_df.loc[row,'droneTimeCost'])
                        operation.truckTimeCost = int(operatiob_df.loc[row,'truckTimeCost'])
                        operation.operationTimeCost = int(operatiob_df.loc[row,'operationTimeCost'])
                        operation.droneServiceID = []  # 出车操作无客户服务
                        operation.truckServiceID = []  # 出车操作无客户服务
                        operation.ServiceID = []  # 该操作无客户服务

                        if str(operatiob_df.loc[row, 'droneServiceID']) == 'nan':
                            pass
                        else:
                            droneServiceLst = str(operatiob_df.loc[row, 'droneServiceID']).split('_')
                            for idx in range(0, len(droneServiceLst)-1):
                                operation.droneServiceID.append(int(droneServiceLst[idx]))

                        if str(operatiob_df.loc[row, 'truckServiceID']) == 'nan':
                            pass
                        else:
                            truckServiceLst = str(operatiob_df.loc[row, 'truckServiceID']).split('_')
                            for idx in range(0, len(truckServiceLst)-1):
                                operation.truckServiceID.append(int(truckServiceLst[idx]))

                        if str(operatiob_df.loc[row, 'ServiceID']) == 'nan':
                            pass
                        else:
                            ServiceLst = str(operatiob_df.loc[row, 'ServiceID']).split('_')
                            for idx in range(0, len(ServiceLst)-1):
                                operation.ServiceID.append(int(ServiceLst[idx]))
                        operation.ServiceDemand = int(operatiob_df.loc[row,'ServiceDemand'])
                        operation.sumOfCustomerPi = int(operatiob_df.loc[row,'sumOfCustomerPi'])
                        operation.ReducedCost = int(operatiob_df.loc[row,'ReducedCost'])
                        OperationDict[operation.key] = operation
                    SP_i.operations = OperationDict
                else:
                    SP_i.operations = copy.deepcopy(node.SP_lst[0].operations)
                    # 复制
            # [dualfix] 不论操作是本轮新建还是已存在，都用当前对偶重算检验数
            # 只需要调整乘子
            for op in SP_i.operations.values():
                # 看这一个操作经过了哪些客户，然后把对应约束[更新后]的乘子求和，最后放到这个操作上去
                piSum1 = 0
                for custerName in op.ServiceID:
                    piSum1 += SP_i.customerPi_DICT[f'{custerName}']
                op.sumOfCustomerPi = piSum1

                op.ReducedCost = op.operationTimeCost - op.sumOfCustomerPi
            # 下面是设计到分支的一些信息，是多个节点之间的更新，根据self.noUse_LST、 self.mustUse_LST删除Operations里面的一些操作，以及调整对应的检验数
            pass  # [silence] print(f"此时子问题{_index}的分支约束键",SP_i.RMPbranchCons_DICT.keys())

            # (一)、根据禁止使用的操作key值，在当前操作字典中找到对应操作，并删除
            for del_op in SP_i.noUse_LST:  # 对于需要删除的操作：del_op
                if del_op in SP_i.operations:
                    del SP_i.operations[del_op]
                    pass  # [silence] print("删除这个操作(不能经过)：", del_op)
                    # SP_i.noUse_LST.remove(del_op)  # 既然已经在.operations中删除了，那就在.noUse_LST中删除，其实就相当于表示已经完成了工作
                else:
                    pass
                    # print(f"需要删除的操作{del_op}竟然不在列表中，需要检查")

            # (二)、根据主问题加的分支约束，来修改对应操作的operation.ReducedCost【存疑】
            # 仔细思考：对于分支约束的禁止使用的操作，其已经被删除了，不可能出现在下次的列中，也就无需对其进行调整
            # 而对于分支约束中必须使用的操作，尽管其一定会出现在下一次的列生成中，应该是需要对其增加一个branchMultiper，保证列检验数计算的完整性
            for branchOperationKey in SP_i.RMPbranchCons_DICT.keys():
                branchMultiper = SP_i.RMPbranchCons_DICT[branchOperationKey]
                if branchOperationKey in SP_i.operations.keys():  # 如果是不能经过，那么已经删除了，没必要调整
                    op = SP_i.operations[branchOperationKey]
                    pass  # [silence] print("分支导致的该操作检验数的变化，(具体而言是必须访问)的那个操作为：", op.key)
                    op.sumOfCustomerPi = op.sumOfCustomerPi + branchMultiper
                    op.ReducedCost -= branchMultiper  # 因为op.ReducedCost = op.operationTimeCost - op.sumOfCustomerPi
                else:
                    pass

            # (三)、根据必须使用的操作key值，在当前操作字典中找到对应操作，利用大数adjustCoff调整其reduceCost，以保证后续一定会使用
            for adjust_op in SP_i.mustUse_LST:  # 对于需要调整的操作：adjust_op
                if adjust_op in SP_i.operations:
                    SP_i.operations[adjust_op].sumOfCustomerPi = SP_i.operations[adjust_op].sumOfCustomerPi + adjustCoff
                    SP_i.operations[adjust_op].ReducedCost -= adjustCoff  # 因为op.ReducedCost = op.operationTimeCost - op.sumOfCustomerPi
                    pass  # [silence] print("利用大数adjustCoff调整这个操作，使其后面最短路中必须经过，这个操作为：", adjust_op)
                else:
                    pass  # [silence] print(f"需要调整的操作{adjust_op}竟然不在列表中，需要检查")

            # 当调整完之后，就可以计算最短路了，只不过还需要对最短路的结果进行一下修正（有调整值的加入，需要去除调整值）
            SPstartTime = time.perf_counter()  # 子问题求解开始时间
            result = dp_SPPVC.shortest_path_operations(SP_i.operations, network, network.vehicleCap[_index],SP_i.capacityPi_DICT)
            SPendTime = time.perf_counter()  # 子问题求解结束时间
            SPtotalTimeCost += SPendTime - SPstartTime
            metrics.REC.add_pricing(SPendTime - SPstartTime)  # [metrics]

            if result is None:
                raise ("问题有误")
            else:
                op = []
                pass  # [silence] print(f"计算得到(考虑客户乘子、分支约束乘子、rounded capacity约束乘子)的最短路成本为：{result[0]}")
                for opUsedIndex in range(0, len(result[1])):  # 当前生成的路径的所有操作集合
                    opUsedKey = result[1][opUsedIndex].key
                    op.append(opUsedKey)
                pass  # [silence] print(f"计算得到的最短路所选择的操作为：{op}")

                SP_i.completionTime = result[1][-1].endTime
                SP_i.reducedCost = result[0] - SP_i.vehiclePi  # 减去车辆服务乘子，这样的话就是考虑了客户服务、车辆服务、分支的乘子、rounded capacity有效不等式后的RC了
                # 对最短路成本 SP_i.reducedCost 进行回调
                for opUsedIndex in range(0, len(result[1])):  # 当前生成的路径的所有操作集合
                    opUsedKey = result[1][opUsedIndex].key
                    if opUsedKey in SP_i.mustUse_LST:
                        SP_i.reducedCost = SP_i.reducedCost + adjustCoff  # 有调整值的加入，需要去除调整值
                pass  # [silence] print(f"回调后当前子问题SP_i的reducedCost为：{SP_i.reducedCost}")
                # 得到具体的哪些客户被什么（卡车、无人机、或者两者共同）来访问了
                truckAlone = []
                droneAlone = []
                together = []
                for i in range(0, len(result[1])):
                    operation = result[1][i]
                    # print(operation.operationName,f"时间从{operation.startTime}到{operation.endTime}，位置从{operation.startID}到{operation.endID}，RC成本是{operation.ReducedCost}")
                    if operation.same == True and len(operation.ServiceID) > 0:  # 说明有客户被卡车和无人机一起访问了(只有C类操作)，添加进together列表
                        together += operation.ServiceID
                    if operation.same == False:  # D类操作或者E类操作
                        truckAlone += operation.truckServiceID
                        droneAlone += operation.droneServiceID
                pass  # [silence] print(f"在这个生成的列（路径）中，卡车访问的客户为：{truckAlone},无人机访问的客户为：{droneAlone},共同访问的客户为：{together}")
                allCustomer = truckAlone+droneAlone+together
                pass  # [silence] print(f"这个生成的列（路径）所服务的客户为{allCustomer}")

                sumRC += round(SP_i.reducedCost, 4)

            if _index == len(node.SP_lst) - 1 and sumRC >= 0:  # 对于最后一个子问题，如果到现在的sum都是0（说明这个最后一个子问题的值也是0），则1.不需要进行列生成，跳出 for 循环（最后一个位置continue也可以实现），2.跳出while循环
                break

            if round(SP_i.reducedCost, 4) >= 0:  # 任何一个检验数大于等于0的子问题，都不需要进行列生成，而是直接跳到下一个
                continue

            Iter += 1
            pass  # [silence] print(f"在主问题完备前，当前列生成的迭代次数：{Iter}，RMP目标函数值为：{node.RMP.ObjVal}，当前子问题{_index}目标函数值（该列的成本）为：{SP_i.reducedCost}")
            pass  # [silence] print("构建列对象并将新列加入RMP")
            # 构建列对象并将新列加入RMP
            col = Column()  # 新建列对象，为了有效地进行列管理
            col_lst = []  # 存放列的一些信息，展示用
            col_lst_SP = []  # 存放某个列经过了哪些操作，这里记录了操作的key值，会用在分支阶段中

            # 生成列
            # (一)、客户服务约束对应的列
            for custIndex in range(0, len(network.Customer)):  # 记录每一个客户都是怎么服务的
                customer_name = network.Customer[custIndex]
                alpha_i = 0
                beta_i = 0
                gama_i = 0
                if customer_name not in allCustomer:  # 该客户没有被这个路径（列）服务
                    coeff = alpha_i + beta_i + gama_i
                    col_lst.append((alpha_i, beta_i, gama_i))
                    col.addTerms(coeff, node.RMP.getConstrByName(f'C_{custIndex}'))
                else:  # 该客户被这个路径（列）服务了（allCustomer是客户名字列表）
                    if customer_name in truckAlone: # 被卡车单独访问
                        alpha_i = 1
                    if customer_name in droneAlone: # 被无人机单独访问
                        beta_i = 1
                    if customer_name in together: # 被卡车和无人机共同访问
                        gama_i = 1
                    coeff = alpha_i + beta_i + gama_i
                    col_lst.append((alpha_i, beta_i, gama_i))
                    col.addTerms(coeff, node.RMP.getConstrByName(f'C_{custIndex}'))

            # (二)、车辆使用约束对应的列
            for i in range(0,len(network.vehicleLst)):  # 车辆路径数量约束
                if i == _index:  # 如果对应的约束就是应当施加在这个子问题（车辆）上面的
                    col.addTerms(1, node.RMP.getConstrByName(f'C_{len(network.Customer)+i}'))  # 异质车辆路径数量约束
                    col_lst.append(1)
                else:
                    col.addTerms(0, node.RMP.getConstrByName(f'C_{len(network.Customer)+i}'))  # 异质车辆路径数量约束
                    col_lst.append(0)

            # (三)、分支约束对应的列
            for i in range(0, len(node.RMPcons)):  # 分支约束  f'NOT_{SP_index}_' + A_[]_i_j_operation.startTime
                op = node.RMPcons[i].split('_')  # 约束名字
                if int(op[1]) == _index:  # 是当前子问题生成的列
                    opKey = f'{op[2]}_{op[3]}_{op[4]}_{op[5]}_{op[6]}'  # 这个分支约束所对应的操作key值
                    # 判断生成的这个路径是否有使用这个操作opName，如果有的话，当前列的该位置就是1，没有的话就是0
                    a_i = 0
                    for opUsedIndex in range(0,len(result[1])):  # 当前生成的路径的所有操作集合
                        opUsedKey = result[1][opUsedIndex].key
                        if opUsedKey == opKey:
                            a_i = 1
                            break
                    col.addTerms(a_i, node.RMP.getConstrByName(node.RMPcons[i]))
                    col_lst.append(a_i)

            # (四)、rounded capacity inequalities生成的列
            for custSubIndex in range(0, len(network.allSubset)):  # 判断当前生成的路径中有没有服务到对应客户子集中的任何一个客户
                custerSubNameSet = network.allSubset[custSubIndex]  # 这个子集中客户名字[12,13]
                # allCustomer是当前路径（列）服务的客户列表
                intersection = set(custerSubNameSet) & set(allCustomer)  # 将这个列经过的客户，与每个子集中的客户取交集
                if len(intersection) !=0:  # 说明有交集，该条约束对应的列位置等于1
                    # print(f"{custerSubNameSet}和{allCustomer}有交集")
                    b_i = 1
                    col_lst.append(b_i)
                    col.addTerms(b_i, node.RMP.getConstrByName(f'capacityCons_{custerSubNameSet}'))
                else:
                    # print(f"{custerSubNameSet}和{allCustomer}无交集")
                    b_i = 0
                    col_lst.append(b_i)
                    col.addTerms(b_i, node.RMP.getConstrByName(f'capacityCons_{custerSubNameSet}'))

            # 生成这个列对应的操作key，用col_lst_SP存储
            for opUsedIndex in range(0, len(result[1])):  # 当前生成的路径的所有操作集合
                opUsedKey = result[1][opUsedIndex].key
                col_lst_SP.append(opUsedKey)

            pass  # [silence] print("生成的列为：", col_lst)
            pass  # [silence] print('生成列的操作详情为：', col_lst_SP)

            cp = SP_i.completionTime  # 此列对应的路由成本（花费的时间）

            node.RMP.addVar(obj=cp, column=col, lb=0, ub=1, name=f"{_index}_newLambda_" + str(Iter))
            node.columnPool[f"{_index}_newLambda_" + str(Iter)] = col_lst  # 随着列生成不断丰富列池
            node.columnPool_SP[f"{_index}_newLambda_" + str(Iter)] = col_lst_SP  # 随着列生成不断丰富列池

        pass  # [silence] print(f'所有子问题RC之和为{sumRC}，相当于表示目前的总Gap为：{-sumRC}')
        if sumRC >= 0:  # [lazy] 定价无改进列：做一次精确分离，有被违反的割就加入并继续列生成
            _tSep = time.perf_counter()
            _nAdd, _newCons = lazyrci.separate(node, network)
            if _nAdd == 0:
                break
            network.allSubset = node.rciSubsets
            node.RMP.update()
            node.RMP.optimize()
            RMPtotalTimeCost += time.perf_counter() - _tSep
            if node.RMP.Status != GRB.OPTIMAL:  # 受限主问题装不下这批割：撤回，本节点不再分离
                lazyrci.rollback(node, _newCons)
                network.allSubset = node.rciSubsets
                node.RMP.update()
                node.RMP.optimize()
                break
            continue

        node.RMP.update()
        RMPstartTime = time.perf_counter()  # 主问题求解开始时间
        node.RMP.optimize()
        RMPendTime = time.perf_counter()  # 主问题求解结束时间
        RMPtotalTimeCost += RMPendTime - RMPstartTime

    for var in node.RMP.getVars():  # 对于RMP解中的任意一个变量
        node.Lambda[var.VarName] = var.x
    pass  # [silence] print('在列生成后，主问题变量node.Lambda有:',node.Lambda)
    pass  # [silence] print("________________________↑列生成结束↑_______________________")
    pass  # [silence] print()
    # node.RMP.write(f"final_RMP_VRPDSTS.lp")

    return node,SPtotalTimeCost,RMPtotalTimeCost


def columnGenerationDP_CUT_PA(node, network,instanceName):  # 输出完备的RMP
    node.RMP.setParam("OutputFlag", 0)
    network.allSubset = node.rciSubsets  # [lazy] 本节点已加入的割
    node.RMP.optimize()
    adjustCoff = 100000000  # 用于让最短路经过某操作的调整值
    # 进入CG循环
    SPtotalTimeCost = 0  # 子问题计算用时
    RMPtotalTimeCost = 0  # 主问题计算用时
    Iter = 0

    cpu_cores = os.cpu_count()  # 线程数量
    coreNum = min(len(node.SP_lst), cpu_cores)
    pool = mppool.get_pool(coreNum, network)  # [mppool] 全进程复用，network 只下发一次

    if node.RMP.Status != GRB.OPTIMAL:  # 由于行的加入，主问题可能无解
        pass  # [silence] print("——————该RMP不可行，割掉当前节点——————")
        return node,SPtotalTimeCost, RMPtotalTimeCost

    pass  # [silence] print("________________________↓开始列生成↓_______________________")
    while (node.RMP.Status == GRB.OPTIMAL):
        if metrics.expired(): break  # [wall] 某个节点的列生成收敛不了时，外层判据永远轮不到
        metrics.REC.d['cg_iters'] += 1  # [metrics]
        for var in node.RMP.getVars():  # 对于RMP解中的任意一个变量
            node.Lambda[var.VarName] = var.x
        pass  # [silence] print('在本次列生成前，主问题变量node.Lambda有:', node.Lambda)

        sumRC = 0  # 初始检验数之和（求解若干子问题）
        Iter_LST = []  # 存放每个子问题命名起点的索引
        input_dataLST = []  # 需要输入并行计算函数中的全部数据
        RC_LST = []  # 检验数列表【输出结果之一】
        operation_LST = []  # 生成的列的具体情况，由若干操作实例组成
        timeCost_LST = []  # 进程用于纯计算所花费的时间
        flag = 0  # 用来判断是否存在结果
        # 将所有子问题的输入集中存储
        for _index in range(0, len(node.SP_lst)):  # 几辆车就有几个子问题，对于这一个子问题而言，需要计算最短路
            SP_i = node.SP_lst[_index]  # 子问题实例
            # 根据主问题变量数量，结合当前子问题情况，来设置生成列的命名起点：（1）先找到属于这一个子问题的列
            prefix = f"{_index}"
            filtered_keys = [key for key in node.Lambda.keys() if key.startswith(prefix)]  # 筛选出这个子问题i下面的所有相关的lambda
            # print('filtered_keys',filtered_keys)
            if len(filtered_keys) == 0:
                Iter = 0
            else:
                a = filtered_keys[-1].split('_')
                if a[1] != 'newLambda':
                    Iter = 0
                elif a[1] == 'newLambda':
                    Iter = int(a[2])
            Iter_LST.append(Iter)

            """___________________针对不同的子问题存储所需要的输入__________________"""
            # 填补此时子问题实例所需要的客户服务乘子 SP_i.customerPi_DICT
            for custIndex in range(0, len(network.Customer)):
                customer_name = network.Customer[custIndex]
                multiper_j = node.RMP.getConstrByName(f'C_{custIndex}').pi
                SP_i.customerPi_DICT[f"{customer_name}"] = multiper_j
            pass  # [silence] print(f"此时子问题{_index}的实例所需要的客户服务乘子:{SP_i.customerPi_DICT}")

            # 填补此时子问题实例所需要的车辆服务乘子 SP_i.vehiclePi
            SP_i.vehiclePi = node.RMP.getConstrByName(f'C_{len(network.Customer) + _index}').pi
            pass  # [silence] print(f"此时子问题{_index}的实例所需要的车辆服务乘子:{SP_i.vehiclePi}")

            # 填补此时子问题实例所需要的分支约束对应的乘子  SP_i.RMPbranchCons_DICT，将对应的操作RC成本加上乘子即可
            for consName in node.RMPcons:  # 这个是在branch过程中向主问题添加的分支约束 f'NOT_{SP_index}_' + A_[]_i_j_operation.startTime
                op = consName.split('_')
                if int(op[1]) == _index:  # 是当前子问题
                    multiper = node.RMP.getConstrByName(consName).pi  # 这条约束的乘子
                    opKey = f'{op[2]}_{op[3]}_{op[4]}_{op[5]}_{op[6]}'  # 该操作的key值
                    SP_i.RMPbranchCons_DICT[opKey] = multiper
            pass  # [silence] print(f"此时子问题{_index}的实例所需要分支约束对应的乘子:{SP_i.RMPbranchCons_DICT}")

            # 填补此时子问题实例所需要的rounded capacity inequalities对应的乘子  SP_i.capacityPi_DICT
            SP_i.capacityPi_DICT = {}  # [lazy] 只保留本节点当前已加入的割的乘子
            for custSubIndex in range(0, len(network.allSubset)):  # 所有的子集都对应有一个约束
                custerSubNameSet = network.allSubset[custSubIndex]
                multiper_B = node.RMP.getConstrByName(f'capacityCons_{custerSubNameSet}').pi
                SP_i.capacityPi_DICT[f"{custerSubNameSet}"] = multiper_B
            pass  # [silence] print(f"此时子问题{_index}的实例所需要的rounded capacity inequalities对应的乘子:{SP_i.capacityPi_DICT}")

            SP_i.reducedCost = 0  # 这一个子问题对应的检验数（=考虑客户乘子的最短路成本+车辆约束乘子+分支约束乘子+后续加强RMP乘子）

            if SP_i.operations == None:  # 若第一个子问题还未构建操作，那就构建
                if _index == 0:  # 是第一个子问题
                    OperationDict = {}
                    # 1.调取Operation并存储
                    operatiob_df = pd.read_csv(
                        fr'{RESULT_DIR}/Performance of acceleration components/{instanceName}/iniOperation.txt',sep="\|",engine="python")
                    for row in range(0, len(operatiob_df)):
                        operation = Operation()
                        operation.startID = int(operatiob_df.loc[row, 'startID'])  # 操作起点
                        operation.startTime = int(operatiob_df.loc[row, 'startTime'])  # 操作开始时间
                        operation.endID = int(operatiob_df.loc[row, 'endID'])  # 操作终点
                        operation.endTime = int(operatiob_df.loc[row, 'endTime'])  # 操作结束时间
                        operation.VehicleId = None
                        operation.operationName = operatiob_df.loc[row, 'operationName']
                        operation.key = operatiob_df.loc[row, 'key']
                        operation.same = operatiob_df.loc[row, 'same']  # 肯定是同一个
                        operation.droneTimeCost = int(operatiob_df.loc[row, 'droneTimeCost'])
                        operation.truckTimeCost = int(operatiob_df.loc[row, 'truckTimeCost'])
                        operation.operationTimeCost = int(operatiob_df.loc[row, 'operationTimeCost'])
                        operation.droneServiceID = []  # 出车操作无客户服务
                        operation.truckServiceID = []  # 出车操作无客户服务
                        operation.ServiceID = []  # 该操作无客户服务

                        if str(operatiob_df.loc[row, 'droneServiceID']) == 'nan':
                            pass
                        else:
                            droneServiceLst = str(operatiob_df.loc[row, 'droneServiceID']).split('_')
                            for idx in range(0, len(droneServiceLst) - 1):
                                operation.droneServiceID.append(int(droneServiceLst[idx]))

                        if str(operatiob_df.loc[row, 'truckServiceID']) == 'nan':
                            pass
                        else:
                            truckServiceLst = str(operatiob_df.loc[row, 'truckServiceID']).split('_')
                            for idx in range(0, len(truckServiceLst) - 1):
                                operation.truckServiceID.append(int(truckServiceLst[idx]))

                        if str(operatiob_df.loc[row, 'ServiceID']) == 'nan':
                            pass
                        else:
                            ServiceLst = str(operatiob_df.loc[row, 'ServiceID']).split('_')
                            for idx in range(0, len(ServiceLst) - 1):
                                operation.ServiceID.append(int(ServiceLst[idx]))
                        operation.ServiceDemand = int(operatiob_df.loc[row, 'ServiceDemand'])
                        operation.sumOfCustomerPi = int(operatiob_df.loc[row, 'sumOfCustomerPi'])
                        operation.ReducedCost = int(operatiob_df.loc[row, 'ReducedCost'])
                        OperationDict[operation.key] = operation
                    SP_i.operations = OperationDict
                else:
                    SP_i.operations = copy.deepcopy(node.SP_lst[0].operations)
                    # 复制
            # [dualfix] 不论操作是本轮新建还是已存在，都用当前对偶重算检验数
            # 只需要调整乘子
            for op in SP_i.operations.values():
                # 看这一个操作经过了哪些客户，然后把对应约束[更新后]的乘子求和，最后放到这个操作上去
                piSum1 = 0
                for custerName in op.ServiceID:
                    piSum1 += SP_i.customerPi_DICT[f'{custerName}']
                op.sumOfCustomerPi = piSum1

                op.ReducedCost = op.operationTimeCost - op.sumOfCustomerPi
            # 下面是设计到分支的一些信息，是多个节点之间的更新，根据self.noUse_LST、 self.mustUse_LST删除Operations里面的一些操作，以及调整对应的检验数
            pass  # [silence] print(f"此时子问题{_index}的分支约束键",SP_i.RMPbranchCons_DICT.keys())

            # (一)、根据禁止使用的操作key值，在当前操作字典中找到对应操作，并删除
            for del_op in SP_i.noUse_LST:  # 对于需要删除的操作：del_op
                if del_op in SP_i.operations:
                    del SP_i.operations[del_op]
                    pass  # [silence] print("删除这个操作(不能经过)：", del_op)
                    # SP_i.noUse_LST.remove(del_op)  # 既然已经在.operations中删除了，那就在.noUse_LST中删除，其实就相当于表示已经完成了工作
                else:
                    pass
                    # print(f"需要删除的操作{del_op}竟然不在列表中，需要检查")

            # (二)、根据主问题加的分支约束，来修改对应操作的operation.ReducedCost【存疑】
            # 仔细思考：对于分支约束的禁止使用的操作，其已经被删除了，不可能出现在下次的列中，也就无需对其进行调整
            # 而对于分支约束中必须使用的操作，尽管其一定会出现在下一次的列生成中，应该是需要对其增加一个branchMultiper，保证列检验数计算的完整性
            for branchOperationKey in SP_i.RMPbranchCons_DICT.keys():
                branchMultiper = SP_i.RMPbranchCons_DICT[branchOperationKey]
                if branchOperationKey in SP_i.operations.keys():  # 如果是不能经过，那么已经删除了，没必要调整
                    op = SP_i.operations[branchOperationKey]
                    pass  # [silence] print("分支导致的该操作检验数的变化，(具体而言是必须访问)的那个操作为：", op.key)
                    op.sumOfCustomerPi = op.sumOfCustomerPi + branchMultiper
                    op.ReducedCost -= branchMultiper  # 因为op.ReducedCost = op.operationTimeCost - op.sumOfCustomerPi
                else:
                    pass

            # (三)、根据必须使用的操作key值，在当前操作字典中找到对应操作，利用大数adjustCoff调整其reduceCost，以保证后续一定会使用
            for adjust_op in SP_i.mustUse_LST:  # 对于需要调整的操作：adjust_op
                if adjust_op in SP_i.operations:
                    SP_i.operations[adjust_op].sumOfCustomerPi = SP_i.operations[adjust_op].sumOfCustomerPi + adjustCoff
                    SP_i.operations[adjust_op].ReducedCost -= adjustCoff  # 因为op.ReducedCost = op.operationTimeCost - op.sumOfCustomerPi
                    pass  # [silence] print("利用大数adjustCoff调整这个操作，使其后面最短路中必须经过，这个操作为：", adjust_op)
                else:
                    pass  # [silence] print(f"需要调整的操作{adjust_op}竟然不在列表中，需要检查")

            input_data = [SP_i.operations, None, network.vehicleCap[_index],SP_i.capacityPi_DICT]  # [mppool] network 已随建池下发
            input_dataLST.append(input_data)

        """___________________已经用input_dataLST存储了若干输入，对其进行并行计算__________________"""
        results = pool.map(dp_SPPVC_p.shortest_path_operations, input_dataLST)  # [mppool]

        for idx, result in enumerate(results):
            RC_LST.append(result[0])
            operation_LST.append(result[1])
            timeCost_LST.append(result[2])
            if result[0] is None:
                flag = 1

        # raise ("stop")

        SPtotalTimeCost += max(timeCost_LST)  # 多线程中，求解时间最长的那个线程，花费的纯计算时间（模型求解+列的生成）
        for _t in timeCost_LST:  # [metrics]
            metrics.REC.add_pricing(_t)
        # 并行计算求解结果，分别是：result(检验数), best_path（操作实例构成的列表）,endTime-startTime（计算时间）

        if flag == 1:
            raise ("问题有误")
        else:
            for _index in range(0, len(node.SP_lst)):  # 几辆车就有几个子问题，对于这一个子问题而言，需要计算最短路
                SP_i = node.SP_lst[_index]  # 子问题实例
                result = results[_index]  # [pafix] 原代码沿用了上一层循环遗留的 result，恒为最后一个子问题的结果

                op = []
                pass  # [silence] print(f"计算得到(考虑客户乘子、分支约束乘子、rounded capacity约束乘子)的最短路成本为：{result[0]}")
                for opUsedIndex in range(0, len(result[1])):  # 当前生成的路径的所有操作集合
                    opUsedKey = result[1][opUsedIndex].key
                    op.append(opUsedKey)
                pass  # [silence] print(f"计算得到的最短路所选择的操作为：{op}")

                SP_i.completionTime = result[1][-1].endTime
                SP_i.reducedCost = result[0] - SP_i.vehiclePi  # 减去车辆服务乘子，这样的话就是考虑了客户服务、车辆服务、分支的乘子、rounded capacity有效不等式后的RC了
                # 对最短路成本 SP_i.reducedCost 进行回调
                for opUsedIndex in range(0, len(result[1])):  # 当前生成的路径的所有操作集合
                    opUsedKey = result[1][opUsedIndex].key
                    if opUsedKey in SP_i.mustUse_LST:
                        SP_i.reducedCost = SP_i.reducedCost + adjustCoff  # 有调整值的加入，需要去除调整值
                pass  # [silence] print(f"回调后当前子问题SP_i的reducedCost为：{SP_i.reducedCost}")
                # 得到具体的哪些客户被什么（卡车、无人机、或者两者共同）来访问了
                truckAlone = []
                droneAlone = []
                together = []
                for i in range(0, len(result[1])):
                    operation = result[1][i]
                    # print(operation.operationName,f"时间从{operation.startTime}到{operation.endTime}，位置从{operation.startID}到{operation.endID}，RC成本是{operation.ReducedCost}")
                    if operation.same == True and len(operation.ServiceID) > 0:  # 说明有客户被卡车和无人机一起访问了(只有C类操作)，添加进together列表
                        together += operation.ServiceID
                    if operation.same == False:  # D类操作或者E类操作
                        truckAlone += operation.truckServiceID
                        droneAlone += operation.droneServiceID
                pass  # [silence] print(f"在这个生成的列（路径）中，卡车访问的客户为：{truckAlone},无人机访问的客户为：{droneAlone},共同访问的客户为：{together}")
                allCustomer = truckAlone+droneAlone+together
                pass  # [silence] print(f"这个生成的列（路径）所服务的客户为{allCustomer}")

                sumRC += round(SP_i.reducedCost, 4)

                if _index == len(node.SP_lst) - 1 and sumRC >= 0:  # 对于最后一个子问题，如果到现在的sum都是0（说明这个最后一个子问题的值也是0），则1.不需要进行列生成，跳出 for 循环（最后一个位置continue也可以实现），2.跳出while循环
                    break

                if round(SP_i.reducedCost, 4) >= 0:  # 任何一个检验数大于等于0的子问题，都不需要进行列生成，而是直接跳到下一个
                    continue

                """____________________________列的生成和加入到RMP中____________________________"""

                if RC_LST[_index] < 0:  # 如果对应的列检验数小于等于0，才会生成新列并将其加入到RMP，否则不会生成
                    pass  # [silence] print("将若干个子问题生成的列插入到RMP中")
                    Iter = Iter_LST[_index]
                    Iter += 1
                    pass  # [silence] print(f"在主问题完备前，当前列生成的迭代次数：{Iter}，RMP目标函数值为：{node.RMP.ObjVal}，当前子问题{_index}目标函数值（该列的成本）为：{SP_i.reducedCost}")
                    pass  # [silence] print("构建列对象并将新列加入RMP")

                    # 构建列对象并将新列加入RMP
                    col = Column()  # 新建列对象，为了有效地进行列管理
                    col_lst = []  # 存放列的一些信息，展示用
                    col_lst_SP = []  # 存放某个列经过了哪些操作，这里记录了操作的key值，会用在分支阶段中

                    # 生成列
                    # (一)、客户服务约束对应的列
                    for custIndex in range(0, len(network.Customer)):  # 记录每一个客户都是怎么服务的
                        customer_name = network.Customer[custIndex]
                        alpha_i = 0
                        beta_i = 0
                        gama_i = 0
                        if customer_name not in allCustomer:  # 该客户没有被这个路径（列）服务
                            coeff = alpha_i + beta_i + gama_i
                            col_lst.append((alpha_i, beta_i, gama_i))
                            col.addTerms(coeff, node.RMP.getConstrByName(f'C_{custIndex}'))
                        else:  # 该客户被这个路径（列）服务了（allCustomer是客户名字列表）
                            if customer_name in truckAlone: # 被卡车单独访问
                                alpha_i = 1
                            if customer_name in droneAlone: # 被无人机单独访问
                                beta_i = 1
                            if customer_name in together: # 被卡车和无人机共同访问
                                gama_i = 1
                            coeff = alpha_i + beta_i + gama_i
                            col_lst.append((alpha_i, beta_i, gama_i))
                            col.addTerms(coeff, node.RMP.getConstrByName(f'C_{custIndex}'))

                    # (二)、车辆使用约束对应的列
                    for i in range(0,len(network.vehicleLst)):  # 车辆路径数量约束
                        if i == _index:  # 如果对应的约束就是应当施加在这个子问题（车辆）上面的
                            col.addTerms(1, node.RMP.getConstrByName(f'C_{len(network.Customer)+i}'))  # 异质车辆路径数量约束
                            col_lst.append(1)
                        else:
                            col.addTerms(0, node.RMP.getConstrByName(f'C_{len(network.Customer)+i}'))  # 异质车辆路径数量约束
                            col_lst.append(0)

                    # (三)、分支约束对应的列
                    for i in range(0, len(node.RMPcons)):  # 分支约束  f'NOT_{SP_index}_' + A_[]_i_j_operation.startTime
                        op = node.RMPcons[i].split('_')  # 约束名字
                        if int(op[1]) == _index:  # 是当前子问题生成的列
                            opKey = f'{op[2]}_{op[3]}_{op[4]}_{op[5]}_{op[6]}'  # 这个分支约束所对应的操作key值
                            # 判断生成的这个路径是否有使用这个操作opName，如果有的话，当前列的该位置就是1，没有的话就是0
                            a_i = 0
                            for opUsedIndex in range(0,len(result[1])):  # 当前生成的路径的所有操作集合
                                opUsedKey = result[1][opUsedIndex].key
                                if opUsedKey == opKey:
                                    a_i = 1
                                    break
                            col.addTerms(a_i, node.RMP.getConstrByName(node.RMPcons[i]))
                            col_lst.append(a_i)

                    # (四)、rounded capacity inequalities生成的列
                    for custSubIndex in range(0, len(network.allSubset)):  # 判断当前生成的路径中有没有服务到对应客户子集中的任何一个客户
                        custerSubNameSet = network.allSubset[custSubIndex]  # 这个子集中客户名字[12,13]
                        # allCustomer是当前路径（列）服务的客户列表
                        intersection = set(custerSubNameSet) & set(allCustomer)  # 将这个列经过的客户，与每个子集中的客户取交集
                        if len(intersection) !=0:  # 说明有交集，该条约束对应的列位置等于1
                            # print(f"{custerSubNameSet}和{allCustomer}有交集")
                            b_i = 1
                            col_lst.append(b_i)
                            col.addTerms(b_i, node.RMP.getConstrByName(f'capacityCons_{custerSubNameSet}'))
                        else:
                            # print(f"{custerSubNameSet}和{allCustomer}无交集")
                            b_i = 0
                            col_lst.append(b_i)
                            col.addTerms(b_i, node.RMP.getConstrByName(f'capacityCons_{custerSubNameSet}'))

                    # 生成这个列对应的操作key，用col_lst_SP存储
                    for opUsedIndex in range(0, len(result[1])):  # 当前生成的路径的所有操作集合
                        opUsedKey = result[1][opUsedIndex].key
                        col_lst_SP.append(opUsedKey)

                    pass  # [silence] print("生成的列为：", col_lst)
                    pass  # [silence] print('生成列的操作详情为：', col_lst_SP)

                    cp = SP_i.completionTime  # 此列对应的路由成本（花费的时间）

                    node.RMP.addVar(obj=cp, column=col, lb=0, ub=1, name=f"{_index}_newLambda_" + str(Iter))
                    node.columnPool[f"{_index}_newLambda_" + str(Iter)] = col_lst  # 随着列生成不断丰富列池
                    node.columnPool_SP[f"{_index}_newLambda_" + str(Iter)] = col_lst_SP  # 随着列生成不断丰富列池

        pass  # [silence] print(f'所有子问题RC之和为{sumRC}，相当于表示目前的总Gap为：{-sumRC}')
        if sumRC >= 0:  # [lazy] 定价无改进列：做一次精确分离，有被违反的割就加入并继续列生成
            _tSep = time.perf_counter()
            _nAdd, _newCons = lazyrci.separate(node, network)
            if _nAdd == 0:
                break
            network.allSubset = node.rciSubsets
            node.RMP.update()
            node.RMP.optimize()
            RMPtotalTimeCost += time.perf_counter() - _tSep
            if node.RMP.Status != GRB.OPTIMAL:  # 受限主问题装不下这批割：撤回，本节点不再分离
                lazyrci.rollback(node, _newCons)
                network.allSubset = node.rciSubsets
                node.RMP.update()
                node.RMP.optimize()
                break
            continue

        node.RMP.update()
        RMPstartTime = time.perf_counter()  # 主问题求解开始时间
        node.RMP.optimize()
        RMPendTime = time.perf_counter()  # 主问题求解结束时间
        RMPtotalTimeCost += RMPendTime - RMPstartTime

    for var in node.RMP.getVars():  # 对于RMP解中的任意一个变量
        node.Lambda[var.VarName] = var.x
    pass  # [silence] print('在列生成后，主问题变量node.Lambda有:',node.Lambda)
    pass  # [silence] print("________________________↑列生成结束↑_______________________")
    pass  # [silence] print()
    # node.RMP.write(f"final_RMP_VRPDSTS.lp")

    return node,SPtotalTimeCost,RMPtotalTimeCost


def branchAndPriceDP(iniNode, eps, globalUB, globalLB, network, instanceName):  # 初始节点
    timeLimit = float("inf")  # [wall] 真正的时限见下一行，原判据只累加纯计算时间
    metrics.DEADLINE = time.perf_counter() + 10800  # [wall]
    nodeLst = [iniNode]  # BB tree node list
    incumbent = None  # 最优解
    global_UB_lst = [globalUB]
    global_LB_lst = [globalLB]

    SPtimeCostInBP = 0  # 分支定价中的子问题总求解时间
    RMPtimeCostInBP = 0  # 分支定价中的主问题总求解时间

    while globalUB - globalLB > eps:
        if metrics.expired(): break  # [wall] 壁钟到点即停
        pass  # [silence] print('分支树中的nodeLst', nodeLst)
        currentNode = nodeLst.pop()
        metrics.REC.d['bb_nodes'] += 1  # [metrics]
        pass  # [silence] print('所选择的currentNode', currentNode)

        currentNode, SPtimeCost, RMPtimeCost = columnGenerationDP(currentNode, network, instanceName)
        SPtimeCostInBP += SPtimeCost
        RMPtimeCostInBP += RMPtimeCost

        is_integer = True  # 后续只要有一个分量不是整数，都会使其变为False，说明当前这个节点（问题）不是整数解
        is_Pruned = False  # 后续只要有一个符合剪枝条件，都会使其变为True，说明当前这个节点（问题）被剪枝了

        if currentNode.RMP.Status == GRB.OPTIMAL:  # 对于当前节点的RMP，如果求得最优解
            # print('列生成后，当前节点的主问题变量currentNode.Lambda:',currentNode.Lambda)

            for var in currentNode.RMP.getVars():  # 对于解中的任意一个变量
                # print(var.VarName, '=', var.x)
                currentNode.Lambda[var.VarName] = var.x
                if abs(var.x - int(var.x)) > eps:  # 说明不是整数解，将这一个解放入候选分支变量集合中
                    is_integer = False
                    currentNode.noInteger_var_lst.append(var.VarName)
                else:
                    currentNode.int_lambda[var.VarName] = int(var.x)
            pass  # [silence] print("非整数解为(待分支变量为)：", currentNode.noInteger_var_lst)
            valueRMP = currentNode.RMP.ObjVal  # 列生成之后的最终主问题的目标函数值（松弛目标函数值）
            if metrics.REC.root_lp is None and not metrics.expired():  # [metrics]  # [wall] 超时中断的根节点不记界
                metrics.REC.root_lp = valueRMP
                metrics.REC.n_rci = sum(1 for _c in currentNode.RMP.getConstrs() if _c.ConstrName.startswith('capacityCons_'))  # [rci]
                metrics.REC.root_rci_active = sum(
                    1 for _c in currentNode.RMP.getConstrs()
                    if _c.ConstrName.startswith('capacityCons_') and abs(_c.Pi) > 1e-9)
            pass  # [silence] print("列生成后RMP函数值：", valueRMP)

            if is_integer == True:  # 是整数解
                if globalUB > valueRMP:  # 可以让全局上界下降的
                    globalUB = valueRMP
                    currentNode.localUB = valueRMP
                    currentNode.localLB = valueRMP
                    incumbent = currentNode  # 最优节点
                else:  # 没能让全局上界下降的
                    currentNode.localUB = valueRMP
                    currentNode.localLB = valueRMP
                is_Pruned = True  # （一）、最优性剪枝

            if is_integer == False:  # 说明有非整数分量，解不是整数解，更新局部下界（松弛解）/局部上界（可行解）
                if globalUB < valueRMP:
                    currentNode.localUB = np.inf
                    currentNode.localLB = valueRMP
                    is_Pruned = True  # （二）、界限剪枝
        else:  # 节点未得到可行解，剪枝
            is_integer = False
            is_Pruned = True

        """ ———————————— 进行分支[分支定价关键的地方]———————————— """
        if is_Pruned == False:  # 说明当前节点没有被剪枝，可以分支
            # 找出非整数Lambda变量所涉及到的子问题，因为这关系到分支约束中，对于子问题那部分的约束信息具体要加到哪个子问题上
            SPlst = []  # 记录哪些涉及到的子问题索引
            for noIntegerVarName in currentNode.noInteger_var_lst:
                a = noIntegerVarName.split('_')
                SPlst.append(int(a[0]))
            SPlst = list(dict.fromkeys(SPlst))  # 去重  # [detfix] 保持首次出现顺序

            # 对于所涉及到的子问题，分别找到其最接近0.5的operationKey，及其流量
            SPoperationRecord = {}  # 一共len(SPlst)数量个键值对，每个都是对应序号子问题中最接近0.5的operationName及其与0.5的距离
            for SP_index in SPlst:
                # 在这一个子问题下，找出累计流量最接近0.5的操作
                operation_flow = {}  # 创建一个记录所有操作流量的字典，然后遍历非整数变量，累加即可
                operation_set = []  # 用来存放所有使用过的操作
                for noIntegerVarName in currentNode.noInteger_var_lst:
                    if int(noIntegerVarName.split('_')[0]) == SP_index:  # 如果是这个问题(因为不关注其他子问题，只记录当前子问题)
                        operation_record = currentNode.columnPool_SP[
                            noIntegerVarName]  # 这是这个非整数变量所经历的操作(含A\B\C\D\E五种操作，眉毛胡子一把抓)
                        for operationKey in operation_record:  # 注意这里的操作存放的名字是其key值：f"E_{customerID_list}_{originS}_{destinationS}_{originT}"
                            operation_set.append(operationKey)
                operation_set = _branch_order(operation_set)  # 去重  # [detfix] 固定次序，分支选择不再依赖哈希种子
                for name in operation_set:
                    operation_flow[name] = 0  # 初始赋值
                for noIntegerVarName in currentNode.noInteger_var_lst:
                    if int(noIntegerVarName.split('_')[0]) == SP_index:
                        noIntegervalue = currentNode.Lambda[noIntegerVarName]  # 非整数变量的值
                        operation_record = currentNode.columnPool_SP[noIntegerVarName]
                        # print(f"非整数变量{noIntegerVarName}所经历的操作", operation_record)
                        for operationKey in operation_record:  # 每个操作的名称
                            operation_flow[operationKey] += noIntegervalue
                # print("当前子问题下，非整数解对应的操作累计流量", operation_flow)

                # 找出当前子问题的累计流量最接近0.5的操作名字即可
                branchOperationKey, branchOperationFlow = next(iter(operation_flow.items()))  # 字典中的第一项
                distance_05 = abs(branchOperationFlow - int(branchOperationFlow) - 0.5)  # 第一项与0.5的距离
                for operationKey, operationFlow in operation_flow.items():
                    distance_var = abs(operationFlow - int(operationFlow) - 0.5)  # 这一个变量和0.5的距离
                    if distance_var < distance_05:  # 如果这一个更加接近0.5
                        branchOperationKey = operationKey
                        distance_05 = distance_var
                SPoperationRecord[f"{SP_index}_" + branchOperationKey] = distance_05
                # print("根据最不可行分支规则，确定当前子问题分支操作的变量名称为",branchOperationKey,"与0.5的距离为：",distance_05)

            # 在字典SParcRecord中，找出来流量最接近0.5的那个operationKey，作为操作分支变量
            SPbranchOperationKey, branchOperationDis05 = next(iter(SPoperationRecord.items()))  # 字典中的第一项
            for operationKey, operationDis05 in SPoperationRecord.items():
                if operationDis05 < branchOperationDis05:
                    branchOperationDis05 = operationDis05
                    SPbranchOperationKey = operationKey  # 1_A_[]_i_j_operation.startTime

            branchSPindex = int(SPbranchOperationKey.split("_")[0])  # 分支操作变量对应的子问题索引
            branchOperationKey = SPbranchOperationKey.split("_")[1] + "_" + SPbranchOperationKey.split("_")[2] + "_" + \
                                 SPbranchOperationKey.split("_")[3] + "_" + SPbranchOperationKey.split("_")[4] + "_" + \
                                 SPbranchOperationKey.split("_")[5]  # A_[]_i_j_operation.startTime
            print(
                f"根据最不可行分支规则，确定分支操作为{branchOperationKey}，所在的子问题索引为{branchSPindex}，距离0.5为{branchOperationDis05}")

            # 根据分支，创建子节点
            leftNode = Node.deepcopy(currentNode)  # 剔除这个基
            rightNode = Node.deepcopy(currentNode)  # 保留这个基

            # 确定分支变量对应的子问题
            SP_index = branchSPindex  # 第几个子问题

            # 确定对应子问题SP_index中哪些lambda经过了这个操作，他们的名字叫什么,筛选出包含给定值的元素的名字
            matching_keys = [key for key, value_list in currentNode.columnPool_SP.items() if
                             branchOperationKey in value_list]  # 这个是所有经过这个操作的子问题索引，多了，要筛选出等于SP_index的弧
            pass  # [silence] print("这些path经过了branchOperation，他们的名字是：", matching_keys)
            newMatching_keys = []
            for lambdaName in matching_keys:
                if int(lambdaName.split("_")[0]) == SP_index:
                    newMatching_keys.append(lambdaName)
            pass  # [silence] print("在这个所针对的子问题中，经过这个分支操作的path为：", newMatching_keys)

            """ 更新对应子节点的RMP,SP——leftNode：令弧上流量为0 """
            # 更新左节点的RMP(令弧上流量为0)
            expr = 0
            for lambdaName in newMatching_keys:
                expr += leftNode.RMP.getVarByName(lambdaName)
            leftNode.RMP.addConstr(expr == 0, f'NOT_{SP_index}_' + branchOperationKey)  # 增加的分支约束
            leftNode.RMPcons.append(f'NOT_{SP_index}_' + branchOperationKey)  # 用这个RMPcons保存一下
            leftNode.RMP.setParam('OutputFlag', 0)
            leftNode.RMP.update()
            # 更新左节点的SP(记录下禁止使用的操作，后续在列生成中所构建的操作集合中不断删除)
            leftNode.SP_lst[SP_index].noUse_LST.append(branchOperationKey)  # 记录下需要删除的操作的key值

            """ 更新对应子节点的RMP,SP——rightNode：令弧上流量为1 """
            # 更新右节点的RMP(令弧上流量为1)
            expr = 0
            for lambdaName in newMatching_keys:
                expr += rightNode.RMP.getVarByName(lambdaName)
            rightNode.RMP.addConstr(expr == 1, f'YES_{SP_index}_' + branchOperationKey)  # 增加的分支约束
            rightNode.RMPcons.append(f'YES_{SP_index}_' + branchOperationKey)  # 用这个RMPcons保存一下
            rightNode.RMP.setParam('OutputFlag', 0)
            rightNode.RMP.update()
            # 更新右节点的SP(记录下必须使用的操作，后续在列生成中所构建的操作集合使其乘子变为较大负值)
            rightNode.SP_lst[SP_index].mustUse_LST.append(branchOperationKey)  # 记录下需要调整的操作的key值

            leftNode.localLB = valueRMP  # [metrics]
            rightNode.localLB = valueRMP  # [metrics]
            nodeLst.append(leftNode)
            nodeLst.append(rightNode)

        # 获得所有叶子节点的局部下界
        localLB_Lst = [globalUB]
        for node in nodeLst:
            localLB_Lst.append(node.localLB)
        localLB_temp = min(localLB_Lst)
        if localLB_temp > globalLB:
            globalLB = localLB_temp

        global_UB_lst.append(globalUB)
        global_LB_lst.append(globalLB)

    return incumbent, global_UB_lst, global_LB_lst, SPtimeCostInBP, RMPtimeCostInBP


def branchAndPriceDP_CUT(iniNode, eps, globalUB, globalLB, network, instanceName):  # 初始节点
    timeLimit = float("inf")  # [wall] 真正的时限见下一行，原判据只累加纯计算时间
    metrics.DEADLINE = time.perf_counter() + 10800  # [wall]

    nodeLst = [iniNode]  # BB tree node list
    incumbent = None  # 最优解
    global_UB_lst = [globalUB]
    global_LB_lst = [globalLB]

    SPtimeCostInBP = 0  # 分支定价中的子问题总求解时间
    RMPtimeCostInBP = 0  # 分支定价中的主问题总求解时间

    while globalUB - globalLB > eps and (SPtimeCostInBP+RMPtimeCostInBP) <= timeLimit:
        if metrics.expired(): break  # [wall] 壁钟到点即停
        pass  # [silence] print('分支树中的nodeLst', nodeLst)
        currentNode = nodeLst.pop()
        metrics.REC.d['bb_nodes'] += 1  # [metrics]
        pass  # [silence] print('所选择的currentNode', currentNode)

        currentNode, SPtimeCost, RMPtimeCost = columnGenerationDP_CUT(currentNode, network, instanceName)
        SPtimeCostInBP += SPtimeCost
        RMPtimeCostInBP += RMPtimeCost

        is_integer = True  # 后续只要有一个分量不是整数，都会使其变为False，说明当前这个节点（问题）不是整数解
        is_Pruned = False  # 后续只要有一个符合剪枝条件，都会使其变为True，说明当前这个节点（问题）被剪枝了

        if currentNode.RMP.Status == GRB.OPTIMAL:  # 对于当前节点的RMP，如果求得最优解
            # print('列生成后，当前节点的主问题变量currentNode.Lambda:',currentNode.Lambda)

            for var in currentNode.RMP.getVars():  # 对于解中的任意一个变量
                # print(var.VarName, '=', var.x)
                currentNode.Lambda[var.VarName] = var.x
                if abs(var.x - int(var.x)) > eps:  # 说明不是整数解，将这一个解放入候选分支变量集合中
                    is_integer = False
                    currentNode.noInteger_var_lst.append(var.VarName)
                else:
                    currentNode.int_lambda[var.VarName] = int(var.x)
            pass  # [silence] print("非整数解为(待分支变量为)：", currentNode.noInteger_var_lst)
            valueRMP = currentNode.RMP.ObjVal  # 列生成之后的最终主问题的目标函数值（松弛目标函数值）
            if metrics.REC.root_lp is None and not metrics.expired():  # [metrics]  # [wall] 超时中断的根节点不记界
                metrics.REC.root_lp = valueRMP
                metrics.REC.n_rci = sum(1 for _c in currentNode.RMP.getConstrs() if _c.ConstrName.startswith('capacityCons_'))  # [rci]
                metrics.REC.root_rci_active = sum(
                    1 for _c in currentNode.RMP.getConstrs()
                    if _c.ConstrName.startswith('capacityCons_') and abs(_c.Pi) > 1e-9)
            pass  # [silence] print("列生成后RMP函数值：", valueRMP)

            if is_integer == True:  # 是整数解
                if globalUB > valueRMP:  # 可以让全局上界下降的
                    globalUB = valueRMP
                    currentNode.localUB = valueRMP
                    currentNode.localLB = valueRMP
                    incumbent = currentNode  # 最优节点
                else:  # 没能让全局上界下降的
                    currentNode.localUB = valueRMP
                    currentNode.localLB = valueRMP
                is_Pruned = True  # （一）、最优性剪枝

            if is_integer == False:  # 说明有非整数分量，解不是整数解，更新局部下界（松弛解）/局部上界（可行解）
                if globalUB < valueRMP:
                    currentNode.localUB = np.inf
                    currentNode.localLB = valueRMP
                    is_Pruned = True  # （二）、界限剪枝
        else:  # 节点未得到可行解，剪枝
            is_integer = False
            is_Pruned = True

        """ ———————————— 进行分支[分支定价关键的地方]———————————— """
        if is_Pruned == False:  # 说明当前节点没有被剪枝，可以分支
            # 找出非整数Lambda变量所涉及到的子问题，因为这关系到分支约束中，对于子问题那部分的约束信息具体要加到哪个子问题上
            SPlst = []  # 记录哪些涉及到的子问题索引
            for noIntegerVarName in currentNode.noInteger_var_lst:
                a = noIntegerVarName.split('_')
                SPlst.append(int(a[0]))
            SPlst = list(dict.fromkeys(SPlst))  # 去重  # [detfix] 保持首次出现顺序

            # 对于所涉及到的子问题，分别找到其最接近0.5的operationKey，及其流量
            SPoperationRecord = {}  # 一共len(SPlst)数量个键值对，每个都是对应序号子问题中最接近0.5的operationName及其与0.5的距离
            for SP_index in SPlst:
                # 在这一个子问题下，找出累计流量最接近0.5的操作
                operation_flow = {}  # 创建一个记录所有操作流量的字典，然后遍历非整数变量，累加即可
                operation_set = []  # 用来存放所有使用过的操作
                for noIntegerVarName in currentNode.noInteger_var_lst:
                    if int(noIntegerVarName.split('_')[0]) == SP_index:  # 如果是这个问题(因为不关注其他子问题，只记录当前子问题)
                        operation_record = currentNode.columnPool_SP[
                            noIntegerVarName]  # 这是这个非整数变量所经历的操作(含A\B\C\D\E五种操作，眉毛胡子一把抓)
                        for operationKey in operation_record:  # 注意这里的操作存放的名字是其key值：f"E_{customerID_list}_{originS}_{destinationS}_{originT}"
                            operation_set.append(operationKey)
                operation_set = _branch_order(operation_set)  # 去重  # [detfix] 固定次序，分支选择不再依赖哈希种子
                for name in operation_set:
                    operation_flow[name] = 0  # 初始赋值
                for noIntegerVarName in currentNode.noInteger_var_lst:
                    if int(noIntegerVarName.split('_')[0]) == SP_index:
                        noIntegervalue = currentNode.Lambda[noIntegerVarName]  # 非整数变量的值
                        operation_record = currentNode.columnPool_SP[noIntegerVarName]
                        # print(f"非整数变量{noIntegerVarName}所经历的操作", operation_record)
                        for operationKey in operation_record:  # 每个操作的名称
                            operation_flow[operationKey] += noIntegervalue
                # print("当前子问题下，非整数解对应的操作累计流量", operation_flow)

                # 找出当前子问题的累计流量最接近0.5的操作名字即可
                branchOperationKey, branchOperationFlow = next(iter(operation_flow.items()))  # 字典中的第一项
                distance_05 = abs(branchOperationFlow - int(branchOperationFlow) - 0.5)  # 第一项与0.5的距离
                for operationKey, operationFlow in operation_flow.items():
                    distance_var = abs(operationFlow - int(operationFlow) - 0.5)  # 这一个变量和0.5的距离
                    if distance_var < distance_05:  # 如果这一个更加接近0.5
                        branchOperationKey = operationKey
                        distance_05 = distance_var
                SPoperationRecord[f"{SP_index}_" + branchOperationKey] = distance_05
                # print("根据最不可行分支规则，确定当前子问题分支操作的变量名称为",branchOperationKey,"与0.5的距离为：",distance_05)

            # 在字典SParcRecord中，找出来流量最接近0.5的那个operationKey，作为操作分支变量
            SPbranchOperationKey, branchOperationDis05 = next(iter(SPoperationRecord.items()))  # 字典中的第一项
            for operationKey, operationDis05 in SPoperationRecord.items():
                if operationDis05 < branchOperationDis05:
                    branchOperationDis05 = operationDis05
                    SPbranchOperationKey = operationKey  # 1_A_[]_i_j_operation.startTime

            branchSPindex = int(SPbranchOperationKey.split("_")[0])  # 分支操作变量对应的子问题索引
            branchOperationKey = SPbranchOperationKey.split("_")[1] + "_" + SPbranchOperationKey.split("_")[2] + "_" + \
                                 SPbranchOperationKey.split("_")[3] + "_" + SPbranchOperationKey.split("_")[4] + "_" + \
                                 SPbranchOperationKey.split("_")[5]  # A_[]_i_j_operation.startTime
            print(
                f"根据最不可行分支规则，确定分支操作为{branchOperationKey}，所在的子问题索引为{branchSPindex}，距离0.5为{branchOperationDis05}")

            # 根据分支，创建子节点
            leftNode = Node.deepcopy(currentNode)  # 剔除这个基
            rightNode = Node.deepcopy(currentNode)  # 保留这个基

            # 确定分支变量对应的子问题
            SP_index = branchSPindex  # 第几个子问题

            # 确定对应子问题SP_index中哪些lambda经过了这个操作，他们的名字叫什么,筛选出包含给定值的元素的名字
            matching_keys = [key for key, value_list in currentNode.columnPool_SP.items() if
                             branchOperationKey in value_list]  # 这个是所有经过这个操作的子问题索引，多了，要筛选出等于SP_index的弧
            pass  # [silence] print("这些path经过了branchOperation，他们的名字是：", matching_keys)
            newMatching_keys = []
            for lambdaName in matching_keys:
                if int(lambdaName.split("_")[0]) == SP_index:
                    newMatching_keys.append(lambdaName)
            pass  # [silence] print("在这个所针对的子问题中，经过这个分支操作的path为：", newMatching_keys)

            """ 更新对应子节点的RMP,SP——leftNode：令弧上流量为0 """
            # 更新左节点的RMP(令弧上流量为0)
            expr = 0
            for lambdaName in newMatching_keys:
                expr += leftNode.RMP.getVarByName(lambdaName)
            leftNode.RMP.addConstr(expr == 0, f'NOT_{SP_index}_' + branchOperationKey)  # 增加的分支约束
            leftNode.RMPcons.append(f'NOT_{SP_index}_' + branchOperationKey)  # 用这个RMPcons保存一下
            leftNode.RMP.setParam('OutputFlag', 0)
            leftNode.RMP.update()
            # 更新左节点的SP(记录下禁止使用的操作，后续在列生成中所构建的操作集合中不断删除)
            leftNode.SP_lst[SP_index].noUse_LST.append(branchOperationKey)  # 记录下需要删除的操作的key值

            """ 更新对应子节点的RMP,SP——rightNode：令弧上流量为1 """
            # 更新右节点的RMP(令弧上流量为1)
            expr = 0
            for lambdaName in newMatching_keys:
                expr += rightNode.RMP.getVarByName(lambdaName)
            rightNode.RMP.addConstr(expr == 1, f'YES_{SP_index}_' + branchOperationKey)  # 增加的分支约束
            rightNode.RMPcons.append(f'YES_{SP_index}_' + branchOperationKey)  # 用这个RMPcons保存一下
            rightNode.RMP.setParam('OutputFlag', 0)
            rightNode.RMP.update()
            # 更新右节点的SP(记录下必须使用的操作，后续在列生成中所构建的操作集合使其乘子变为较大负值)
            rightNode.SP_lst[SP_index].mustUse_LST.append(branchOperationKey)  # 记录下需要调整的操作的key值

            leftNode.localLB = valueRMP  # [metrics]
            rightNode.localLB = valueRMP  # [metrics]
            nodeLst.append(leftNode)
            nodeLst.append(rightNode)

        # 获得所有叶子节点的局部下界
        localLB_Lst = [globalUB]
        for node in nodeLst:
            localLB_Lst.append(node.localLB)
        localLB_temp = min(localLB_Lst)
        if localLB_temp > globalLB:
            globalLB = localLB_temp

        global_UB_lst.append(globalUB)
        global_LB_lst.append(globalLB)

    return incumbent, global_UB_lst, global_LB_lst, SPtimeCostInBP, RMPtimeCostInBP


def branchAndPriceDP_CUT_PA(iniNode, eps, globalUB, globalLB, network, instanceName):  # 初始节点
    timeLimit = float("inf")  # [wall] 真正的时限见下一行，原判据只累加纯计算时间
    metrics.DEADLINE = time.perf_counter() + 10800  # [wall]

    nodeLst = [iniNode]  # BB tree node list
    incumbent = None  # 最优解
    global_UB_lst = [globalUB]
    global_LB_lst = [globalLB]

    SPtimeCostInBP = 0  # 分支定价中的子问题总求解时间
    RMPtimeCostInBP = 0  # 分支定价中的主问题总求解时间

    while globalUB - globalLB > eps and (SPtimeCostInBP+RMPtimeCostInBP) <= timeLimit:
        if metrics.expired(): break  # [wall] 壁钟到点即停

        pass  # [silence] print('分支树中的nodeLst', nodeLst)
        currentNode = nodeLst.pop()
        metrics.REC.d['bb_nodes'] += 1  # [metrics]
        pass  # [silence] print('所选择的currentNode', currentNode)

        currentNode, SPtimeCost, RMPtimeCost = columnGenerationDP_CUT_PA(currentNode, network, instanceName)
        SPtimeCostInBP += SPtimeCost
        RMPtimeCostInBP += RMPtimeCost

        is_integer = True  # 后续只要有一个分量不是整数，都会使其变为False，说明当前这个节点（问题）不是整数解
        is_Pruned = False  # 后续只要有一个符合剪枝条件，都会使其变为True，说明当前这个节点（问题）被剪枝了

        if currentNode.RMP.Status == GRB.OPTIMAL:  # 对于当前节点的RMP，如果求得最优解
            # print('列生成后，当前节点的主问题变量currentNode.Lambda:',currentNode.Lambda)

            for var in currentNode.RMP.getVars():  # 对于解中的任意一个变量
                # print(var.VarName, '=', var.x)
                currentNode.Lambda[var.VarName] = var.x
                if abs(var.x - int(var.x)) > eps:  # 说明不是整数解，将这一个解放入候选分支变量集合中
                    is_integer = False
                    currentNode.noInteger_var_lst.append(var.VarName)
                else:
                    currentNode.int_lambda[var.VarName] = int(var.x)
            pass  # [silence] print("非整数解为(待分支变量为)：", currentNode.noInteger_var_lst)
            valueRMP = currentNode.RMP.ObjVal  # 列生成之后的最终主问题的目标函数值（松弛目标函数值）
            if metrics.REC.root_lp is None and not metrics.expired():  # [metrics]  # [wall] 超时中断的根节点不记界
                metrics.REC.root_lp = valueRMP
                metrics.REC.n_rci = sum(1 for _c in currentNode.RMP.getConstrs() if _c.ConstrName.startswith('capacityCons_'))  # [rci]
                metrics.REC.root_rci_active = sum(
                    1 for _c in currentNode.RMP.getConstrs()
                    if _c.ConstrName.startswith('capacityCons_') and abs(_c.Pi) > 1e-9)
            pass  # [silence] print("列生成后RMP函数值：", valueRMP)

            if is_integer == True:  # 是整数解
                if globalUB > valueRMP:  # 可以让全局上界下降的
                    globalUB = valueRMP
                    currentNode.localUB = valueRMP
                    currentNode.localLB = valueRMP
                    incumbent = currentNode  # 最优节点
                else:  # 没能让全局上界下降的
                    currentNode.localUB = valueRMP
                    currentNode.localLB = valueRMP
                is_Pruned = True  # （一）、最优性剪枝

            if is_integer == False:  # 说明有非整数分量，解不是整数解，更新局部下界（松弛解）/局部上界（可行解）
                if globalUB < valueRMP:
                    currentNode.localUB = np.inf
                    currentNode.localLB = valueRMP
                    is_Pruned = True  # （二）、界限剪枝
        else:  # 节点未得到可行解，剪枝
            is_integer = False
            is_Pruned = True

        """ ———————————— 进行分支[分支定价关键的地方]———————————— """
        if is_Pruned == False:  # 说明当前节点没有被剪枝，可以分支
            # 找出非整数Lambda变量所涉及到的子问题，因为这关系到分支约束中，对于子问题那部分的约束信息具体要加到哪个子问题上
            SPlst = []  # 记录哪些涉及到的子问题索引
            for noIntegerVarName in currentNode.noInteger_var_lst:
                a = noIntegerVarName.split('_')
                SPlst.append(int(a[0]))
            SPlst = list(dict.fromkeys(SPlst))  # 去重  # [detfix] 保持首次出现顺序

            # 对于所涉及到的子问题，分别找到其最接近0.5的operationKey，及其流量
            SPoperationRecord = {}  # 一共len(SPlst)数量个键值对，每个都是对应序号子问题中最接近0.5的operationName及其与0.5的距离
            for SP_index in SPlst:
                # 在这一个子问题下，找出累计流量最接近0.5的操作
                operation_flow = {}  # 创建一个记录所有操作流量的字典，然后遍历非整数变量，累加即可
                operation_set = []  # 用来存放所有使用过的操作
                for noIntegerVarName in currentNode.noInteger_var_lst:
                    if int(noIntegerVarName.split('_')[0]) == SP_index:  # 如果是这个问题(因为不关注其他子问题，只记录当前子问题)
                        operation_record = currentNode.columnPool_SP[
                            noIntegerVarName]  # 这是这个非整数变量所经历的操作(含A\B\C\D\E五种操作，眉毛胡子一把抓)
                        for operationKey in operation_record:  # 注意这里的操作存放的名字是其key值：f"E_{customerID_list}_{originS}_{destinationS}_{originT}"
                            operation_set.append(operationKey)
                operation_set = _branch_order(operation_set)  # 去重  # [detfix] 固定次序，分支选择不再依赖哈希种子
                for name in operation_set:
                    operation_flow[name] = 0  # 初始赋值
                for noIntegerVarName in currentNode.noInteger_var_lst:
                    if int(noIntegerVarName.split('_')[0]) == SP_index:
                        noIntegervalue = currentNode.Lambda[noIntegerVarName]  # 非整数变量的值
                        operation_record = currentNode.columnPool_SP[noIntegerVarName]
                        # print(f"非整数变量{noIntegerVarName}所经历的操作", operation_record)
                        for operationKey in operation_record:  # 每个操作的名称
                            operation_flow[operationKey] += noIntegervalue
                # print("当前子问题下，非整数解对应的操作累计流量", operation_flow)

                # 找出当前子问题的累计流量最接近0.5的操作名字即可
                branchOperationKey, branchOperationFlow = next(iter(operation_flow.items()))  # 字典中的第一项
                distance_05 = abs(branchOperationFlow - int(branchOperationFlow) - 0.5)  # 第一项与0.5的距离
                for operationKey, operationFlow in operation_flow.items():
                    distance_var = abs(operationFlow - int(operationFlow) - 0.5)  # 这一个变量和0.5的距离
                    if distance_var < distance_05:  # 如果这一个更加接近0.5
                        branchOperationKey = operationKey
                        distance_05 = distance_var
                SPoperationRecord[f"{SP_index}_" + branchOperationKey] = distance_05
                # print("根据最不可行分支规则，确定当前子问题分支操作的变量名称为",branchOperationKey,"与0.5的距离为：",distance_05)

            # 在字典SParcRecord中，找出来流量最接近0.5的那个operationKey，作为操作分支变量
            SPbranchOperationKey, branchOperationDis05 = next(iter(SPoperationRecord.items()))  # 字典中的第一项
            for operationKey, operationDis05 in SPoperationRecord.items():
                if operationDis05 < branchOperationDis05:
                    branchOperationDis05 = operationDis05
                    SPbranchOperationKey = operationKey  # 1_A_[]_i_j_operation.startTime

            branchSPindex = int(SPbranchOperationKey.split("_")[0])  # 分支操作变量对应的子问题索引
            branchOperationKey = SPbranchOperationKey.split("_")[1] + "_" + SPbranchOperationKey.split("_")[2] + "_" + \
                                 SPbranchOperationKey.split("_")[3] + "_" + SPbranchOperationKey.split("_")[4] + "_" + \
                                 SPbranchOperationKey.split("_")[5]  # A_[]_i_j_operation.startTime
            print(
                f"根据最不可行分支规则，确定分支操作为{branchOperationKey}，所在的子问题索引为{branchSPindex}，距离0.5为{branchOperationDis05}")

            # 根据分支，创建子节点
            leftNode = Node.deepcopy(currentNode)  # 剔除这个基
            rightNode = Node.deepcopy(currentNode)  # 保留这个基

            # 确定分支变量对应的子问题
            SP_index = branchSPindex  # 第几个子问题

            # 确定对应子问题SP_index中哪些lambda经过了这个操作，他们的名字叫什么,筛选出包含给定值的元素的名字
            matching_keys = [key for key, value_list in currentNode.columnPool_SP.items() if
                             branchOperationKey in value_list]  # 这个是所有经过这个操作的子问题索引，多了，要筛选出等于SP_index的弧
            pass  # [silence] print("这些path经过了branchOperation，他们的名字是：", matching_keys)
            newMatching_keys = []
            for lambdaName in matching_keys:
                if int(lambdaName.split("_")[0]) == SP_index:
                    newMatching_keys.append(lambdaName)
            pass  # [silence] print("在这个所针对的子问题中，经过这个分支操作的path为：", newMatching_keys)

            """ 更新对应子节点的RMP,SP——leftNode：令弧上流量为0 """
            # 更新左节点的RMP(令弧上流量为0)
            expr = 0
            for lambdaName in newMatching_keys:
                expr += leftNode.RMP.getVarByName(lambdaName)
            leftNode.RMP.addConstr(expr == 0, f'NOT_{SP_index}_' + branchOperationKey)  # 增加的分支约束
            leftNode.RMPcons.append(f'NOT_{SP_index}_' + branchOperationKey)  # 用这个RMPcons保存一下
            leftNode.RMP.setParam('OutputFlag', 0)
            leftNode.RMP.update()
            # 更新左节点的SP(记录下禁止使用的操作，后续在列生成中所构建的操作集合中不断删除)
            leftNode.SP_lst[SP_index].noUse_LST.append(branchOperationKey)  # 记录下需要删除的操作的key值

            """ 更新对应子节点的RMP,SP——rightNode：令弧上流量为1 """
            # 更新右节点的RMP(令弧上流量为1)
            expr = 0
            for lambdaName in newMatching_keys:
                expr += rightNode.RMP.getVarByName(lambdaName)
            rightNode.RMP.addConstr(expr == 1, f'YES_{SP_index}_' + branchOperationKey)  # 增加的分支约束
            rightNode.RMPcons.append(f'YES_{SP_index}_' + branchOperationKey)  # 用这个RMPcons保存一下
            rightNode.RMP.setParam('OutputFlag', 0)
            rightNode.RMP.update()
            # 更新右节点的SP(记录下必须使用的操作，后续在列生成中所构建的操作集合使其乘子变为较大负值)
            rightNode.SP_lst[SP_index].mustUse_LST.append(branchOperationKey)  # 记录下需要调整的操作的key值

            leftNode.localLB = valueRMP  # [metrics]
            rightNode.localLB = valueRMP  # [metrics]
            nodeLst.append(leftNode)
            nodeLst.append(rightNode)

        # 获得所有叶子节点的局部下界
        localLB_Lst = [globalUB]
        for node in nodeLst:
            localLB_Lst.append(node.localLB)
        localLB_temp = min(localLB_Lst)
        if localLB_temp > globalLB:
            globalLB = localLB_temp

        global_UB_lst.append(globalUB)
        global_LB_lst.append(globalLB)

    return incumbent, global_UB_lst, global_LB_lst, SPtimeCostInBP, RMPtimeCostInBP


if __name__ == "__main__":
    # 记得修改constructNetwork的对应网络name
    eps = 0.0001
    globalUB, globalLB = np.inf, 0
    CustomerLst = [25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36]  # 虚拟客户点id
    physicalCustomerList = [4, 12, 14, 18, 2, 13, 6, 16, 11, 10, 15, 19]  # 实际的客户点id
    demand = [4,7,3,5,6,3,6,4,7,5,6,3]  # 客户需求量

    O = 0
    D = 37
    Vehicle_Cap = [55, 55, 55]  # 卡车组容量
    Truckcapacity = [50, 50, 50]  # 卡车容量
    UAVcapacity = [5, 5, 5]  # 无人机容量
    UAVendurance = [15, 15, 15]  # 无人机续航
    vehicleNum = 3  # 车辆数量
    iniBase = [[1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0], [0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 1, 0],
               [0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1]]  # 长度=C个客户+v（车辆数）条约束
    iniCost = [44, 48, 54]

    vehicleLst = [i for i in range(0, vehicleNum)]
    sL = 1
    sR = 1
    sS = 1
    instanceName = 'SFnetwork_12customer'

    st_network = construct12.network
    st_network.readStsNode(physicalCustomerList, CustomerLst, vehicleLst, demand, Vehicle_Cap,Truckcapacity,UAVcapacity, UAVendurance,sL, sR, sS, O, D)
    st_network.readStsVehicleArc()
    st_network.readPotentialSorties(sL, sR)
    st_network.readStsDroneArc()

    print("————————————————（2）分支定价(子问题DP+有效不等式)—————————————————")
    iniNode = modelingDP_CUT(st_network, iniBase, iniCost)
    metrics.REC.reset()  # [metrics]
    start_time = time.perf_counter()
    optimalIntegerSol, global_UB_lst, global_LB_lst, SPtimeCostInBP, RMPtimeCostInBP = branchAndPriceDP_CUT(iniNode,
                                                                                                            eps,
                                                                                                            globalUB,
                                                                                                            globalLB,
                                                                                                            st_network,
                                                                                                            instanceName)
    BP_total = time.perf_counter() - start_time
    # [metrics] 写出本算法的运行统计
    metrics.REC.dump(
        fr'{RESULT_DIR}/Performance of acceleration components/{instanceName}/metrics(DP+Cut).json',
        extra={'instance': instanceName,
               'algorithm': 'DP+Cut',
               'total_time': BP_total,
               'SP_time': SPtimeCostInBP,
               'RMP_time': RMPtimeCostInBP,
               'UB': float(global_UB_lst[-1]),
               'LB': float(global_LB_lst[-1]),
               'gap': (None if not np.isfinite(global_UB_lst[-1])
                       else float(global_UB_lst[-1] - global_LB_lst[-1])
                            / max(abs(float(global_UB_lst[-1])), 1e-9)),
               'proved_optimal': bool(np.isfinite(global_UB_lst[-1])
                                      and global_UB_lst[-1] - global_LB_lst[-1] <= 1e-4)})
    if optimalIntegerSol is None:
        print('!! 时限内未找到整数解，OBJ 记为 inf')
        optimalIntegerSol = metrics.NoSolution()
    print(f"分支定价求解时间: {BP_total} s")
    print(f"分支定价中，子问题总求解时间: {SPtimeCostInBP} s")
    print(f"分支定价中，主问题总求解时间: {RMPtimeCostInBP} s")

    varLst = optimalIntegerSol.RMP.getVars()
    print("RMP目标函数值", optimalIntegerSol.RMP.ObjVal, "RMP决策变量为：")
    for var in varLst:
        if var.X != 0:
            print(var.VarName, var.X)

    print("对应的列池", optimalIntegerSol.columnPool)
    print("对应列池的路由情况", optimalIntegerSol.columnPool_SP)

    print('global_UB_lst', global_UB_lst)
    print('global_LB_lst', global_LB_lst)

    # 记得要新建好对应位置、对应名字的空文件夹
    result_Path = fr'{RESULT_DIR}/Performance of acceleration components/{instanceName}/result(DP+Cut).txt'
    result_File = open(result_Path, 'w').close()
    result_generate = open(result_Path, mode='a')
    result_generate.write(str('SP_time:') + ',' + str(SPtimeCostInBP))
    result_generate.write('\n')
    result_generate.write(str('RMP_time:') + ',' + str(RMPtimeCostInBP))
    result_generate.write('\n')
    result_generate.write(str('TOTAL_time:') + ',' + str(BP_total))
    result_generate.write('\n')
    result_generate.write(str('OBJ_value:') + ',' + str(optimalIntegerSol.RMP.ObjVal))

    result_generate.write('\n\n')
    result_generate.write(str('RMP_variable:'))
    result_generate.write('\n')
    for var in varLst:
        if var.X != 0:
            result_generate.write(str(var.VarName) + ',' + str(var.X))
            result_generate.write('\n')

    result_generate.write('\n')
    result_generate.write(str('UpperBound:'))
    result_generate.write('\n')
    for ub in global_UB_lst:
        result_generate.write(str(ub) + ',')

    result_generate.write('\n\n')
    result_generate.write(str('LowerBound:'))
    result_generate.write('\n')
    for lb in global_LB_lst:
        result_generate.write(str(lb) + ',')
    result_generate.write('\n\n')

    for key, value in optimalIntegerSol.columnPool.items():
        result_generate.write(str(key) + ":," + str(value) + ',')
    result_generate.write('\n')
    result_generate.write('\n')
    for key, value in optimalIntegerSol.columnPool_SP.items():
        result_generate.write(str(key) + ":," + str(value) + ',')
    result_generate.close()

    print("————————————————（3）分支定价(子问题DP+强化割+并行)————————————————")
    iniNode = modelingDP_CUT_PA(st_network, iniBase, iniCost)
    metrics.REC.reset()  # [metrics]
    start_time = time.perf_counter()
    optimalIntegerSol, global_UB_lst, global_LB_lst, SPtimeCostInBP, RMPtimeCostInBP = branchAndPriceDP_CUT_PA(iniNode,
                                                                                                               eps,
                                                                                                               globalUB,
                                                                                                               globalLB,
                                                                                                               st_network,
                                                                                                               instanceName)
    BP_total = time.perf_counter() - start_time
    # [metrics] 写出本算法的运行统计
    metrics.REC.dump(
        fr'{RESULT_DIR}/Performance of acceleration components/{instanceName}/metrics(DP+Cut+parallel).json',
        extra={'instance': instanceName,
               'algorithm': 'DP+Cut+parallel',
               'total_time': BP_total,
               'SP_time': SPtimeCostInBP,
               'RMP_time': RMPtimeCostInBP,
               'UB': float(global_UB_lst[-1]),
               'LB': float(global_LB_lst[-1]),
               'gap': (None if not np.isfinite(global_UB_lst[-1])
                       else float(global_UB_lst[-1] - global_LB_lst[-1])
                            / max(abs(float(global_UB_lst[-1])), 1e-9)),
               'proved_optimal': bool(np.isfinite(global_UB_lst[-1])
                                      and global_UB_lst[-1] - global_LB_lst[-1] <= 1e-4)})
    if optimalIntegerSol is None:
        print('!! 时限内未找到整数解，OBJ 记为 inf')
        optimalIntegerSol = metrics.NoSolution()
    print(f"分支定价求解时间: {BP_total} s")
    print(f"分支定价中，子问题总求解时间: {SPtimeCostInBP} s")
    print(f"分支定价中，主问题总求解时间: {RMPtimeCostInBP} s")

    varLst = optimalIntegerSol.RMP.getVars()
    print("RMP目标函数值", optimalIntegerSol.RMP.ObjVal, "RMP决策变量为：")
    for var in varLst:
        if var.X != 0:
            print(var.VarName, var.X)

    print("对应的列池", optimalIntegerSol.columnPool)
    print("对应列池的路由情况", optimalIntegerSol.columnPool_SP)

    print('global_UB_lst', global_UB_lst)
    print('global_LB_lst', global_LB_lst)

    # 记得要新建好对应位置、对应名字的空文件夹
    result_Path = fr'{RESULT_DIR}/Performance of acceleration components/{instanceName}/result(DP+Cut+parallel).txt'
    result_File = open(result_Path, 'w').close()
    result_generate = open(result_Path, mode='a')
    result_generate.write(str('SP_time:') + ',' + str(SPtimeCostInBP))
    result_generate.write('\n')
    result_generate.write(str('RMP_time:') + ',' + str(RMPtimeCostInBP))
    result_generate.write('\n')
    result_generate.write(str('TOTAL_time:') + ',' + str(BP_total))
    result_generate.write('\n')
    result_generate.write(str('OBJ_value:') + ',' + str(optimalIntegerSol.RMP.ObjVal))

    result_generate.write('\n\n')
    result_generate.write(str('RMP_variable:'))
    result_generate.write('\n')
    for var in varLst:
        if var.X != 0:
            result_generate.write(str(var.VarName) + ',' + str(var.X))
            result_generate.write('\n')

    result_generate.write('\n')
    result_generate.write(str('UpperBound:'))
    result_generate.write('\n')
    for ub in global_UB_lst:
        result_generate.write(str(ub) + ',')

    result_generate.write('\n\n')
    result_generate.write(str('LowerBound:'))
    result_generate.write('\n')
    for lb in global_LB_lst:
        result_generate.write(str(lb) + ',')
    result_generate.write('\n\n')

    for key, value in optimalIntegerSol.columnPool.items():
        result_generate.write(str(key) + ":," + str(value) + ',')
    result_generate.write('\n')
    result_generate.write('\n')
    for key, value in optimalIntegerSol.columnPool_SP.items():
        result_generate.write(str(key) + ":," + str(value) + ',')
    result_generate.close()

    # [restore] 以下算法块由 patch_restore_blocks.py 补回
    print("————————————————（1）分支定价(子问题DP)—————————————————")
    iniNode = modelingDP(st_network, iniBase, iniCost)
    metrics.REC.reset()  # [metrics]
    start_time = time.perf_counter()
    optimalIntegerSol,global_UB_lst, global_LB_lst,SPtimeCostInBP,RMPtimeCostInBP = branchAndPriceDP(iniNode,eps,globalUB,globalLB, st_network,instanceName)
    BP_total = time.perf_counter() - start_time
    # [metrics] 写出本算法的运行统计
    metrics.REC.dump(
        fr'{RESULT_DIR}/Performance of acceleration components/{instanceName}/metrics(DP).json',
        extra={'instance': instanceName,
               'algorithm': 'DP',
               'total_time': BP_total,
               'SP_time': SPtimeCostInBP,
               'RMP_time': RMPtimeCostInBP,
               'UB': float(global_UB_lst[-1]),
               'LB': float(global_LB_lst[-1]),
               'gap': (None if not np.isfinite(global_UB_lst[-1])
                       else float(global_UB_lst[-1] - global_LB_lst[-1])
                            / max(abs(float(global_UB_lst[-1])), 1e-9)),
               'proved_optimal': bool(np.isfinite(global_UB_lst[-1])
                                      and global_UB_lst[-1] - global_LB_lst[-1] <= 1e-4)})
    if optimalIntegerSol is None:
        print('!! 时限内未找到整数解，OBJ 记为 inf')
        optimalIntegerSol = metrics.NoSolution()
    print(f"分支定价求解时间: {BP_total} s")
    print(f"分支定价中，子问题总求解时间: {SPtimeCostInBP} s")
    print(f"分支定价中，主问题总求解时间: {RMPtimeCostInBP} s")

    varLst = optimalIntegerSol.RMP.getVars()
    print("RMP目标函数值", optimalIntegerSol.RMP.ObjVal,"RMP决策变量为：")
    for var in varLst:
        if var.X != 0:
            print(var.VarName, var.X)

    print("对应的列池", optimalIntegerSol.columnPool)
    print("对应列池的路由情况", optimalIntegerSol.columnPool_SP)

    print('global_UB_lst',global_UB_lst)
    print('global_LB_lst',global_LB_lst)

    # 记得要新建好对应位置、对应名字的空文件夹
    result_Path = fr'{RESULT_DIR}/Performance of acceleration components/{instanceName}/result(DP).txt'
    result_File = open(result_Path, 'w').close()
    result_generate = open(result_Path, mode='a')
    result_generate.write(str('SP_time:') + ',' + str(SPtimeCostInBP))
    result_generate.write('\n')
    result_generate.write(str('RMP_time:') + ',' + str(RMPtimeCostInBP))
    result_generate.write('\n')
    result_generate.write(str('TOTAL_time:') + ',' + str(BP_total))
    result_generate.write('\n')
    result_generate.write(str('OBJ_value:') + ',' + str(optimalIntegerSol.RMP.ObjVal))

    result_generate.write('\n\n')
    result_generate.write(str('RMP_variable:'))
    result_generate.write('\n')
    for var in varLst:
        if var.X != 0:
            result_generate.write(str(var.VarName) + ',' + str(var.X))
            result_generate.write('\n')

    result_generate.write('\n')
    result_generate.write(str('UpperBound:'))
    result_generate.write('\n')
    for ub in global_UB_lst:
        result_generate.write(str(ub) + ',')

    result_generate.write('\n\n')
    result_generate.write(str('LowerBound:'))
    result_generate.write('\n')
    for lb in global_LB_lst:
        result_generate.write(str(lb) + ',')
    result_generate.write('\n\n')

    for key, value in optimalIntegerSol.columnPool.items():
        result_generate.write(str(key) + ":," + str(value) + ',')
    result_generate.write('\n')
    result_generate.write('\n')
    for key, value in optimalIntegerSol.columnPool_SP.items():
        result_generate.write(str(key) + ":," + str(value) + ',')
    result_generate.close()

