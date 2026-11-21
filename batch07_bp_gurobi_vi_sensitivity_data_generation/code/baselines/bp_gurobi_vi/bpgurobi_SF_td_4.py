# -*- coding: utf-8 -*-
# B&P-Gurobi-VI for instance timeDependent_SFnetwork_4customer: branch-and-price with the pricing subproblem solved as a MIP by Gurobi and the rounded capacity inequalities added a priori (Tables 6, 7, 10).
# Original file name in the authors' project: 时变-实例计算-SF4节点(1.子问题Gurobi).py (last edited 2025/5/16)
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
from gurobipy import *

# [release] Constraint (33) of the published paper bounds the subset indicator p from above as well as from below
# (p = 1 if and only if the route serves a customer of the subset). The runs reported in Tables 6, 7 and 10 were made
# with the lower bound only (the form of the first submission); set False to reproduce that form exactly.
UPPER_BOUND_ON_P = True
import TDconstructStsNetwork4 as TDconstruct4
import time
import copy
import numpy as np
import math


class Node:
    def __init__(self):
        self.RMP = None
        self.SP_lst = []  # 子问题列表，子问题的数量等于车辆数，每辆车都是一个子问题
        self.X = {}  # SP变量
        self.Y = {}  # SP变量
        self.D = {}  # SP变量
        self.localUB = np.inf
        self.localLB = 0
        self.Lambda = {}  # 主问题RMP的变量的键值对，不是变量字典，是值字典
        self.int_lambda = {}  # 整数变量的键值对
        self.RMPcons = []  # 记录分支过程中，向RMP中所添加的某基系数=0（NOT）或者某基系数=1（YES），所对应的约束名字

        self.branch_var_lst = []  # 分支变量
        self.is_integer = False
        self.columnPool = {}  # 主问题的列池,lambda变量名：[基]
        self.columnPool_SP = {}  # 主问题的列池对应的子问题的解，lambda变量名：[子问题中对应的x=1的自变量]
        self.reduceCost = - np.inf  # 最终的子问题SP的检验数


    def deepcopy(node):
        new_node = Node()  # 实例化
        new_node.RMP = node.RMP.copy()  # 复制后新结点的模型与复制前相同
        new_node.SP_lst = []
        for SP_i in node.SP_lst:
            SP_iNew = SP_i.copy()
            new_node.SP_lst.append(SP_iNew)
        new_node.X = copy.deepcopy(node.X.copy())
        new_node.Y = copy.deepcopy(node.Y.copy())
        new_node.D = copy.deepcopy(node.D.copy())
        new_node.localUB = np.inf
        new_node.localLB = 0
        new_node.Lambda = copy.deepcopy(node.Lambda)
        new_node.RMPcons = copy.deepcopy(node.RMPcons)

        new_node.int_lambda = copy.deepcopy(node.int_lambda)
        new_node.branch_var_lst = []
        new_node.is_integer = node.is_integer
        new_node.columnPool = copy.deepcopy(node.columnPool)
        new_node.columnPool_SP = copy.deepcopy(node.columnPool_SP)
        new_node.reduceCost = - np.inf

        return new_node


def RHSnum(demand, CustomerLst, subSet, vehicleCap):  # 输入所有客户的需求，name列表，客户集合C’，所使用的卡车索引，各个卡车的容量，输出最少的使用数量
    # 从大到小整理出所使用的车的容量

    # 计算客户集合C’的totalDemand
    totalDemand = 0
    for jname in subSet:  # C'中的客户name
        jindex = CustomerLst.index(jname)
        totalDemand += demand[jindex]
    print("totalDemand:", totalDemand)
    # 开始计数
    maxCap = max(vehicleCap)
    cnt = math.ceil(totalDemand/maxCap)
    return cnt


def modeling(network, iniBase, iniCost):  # 输出初始的受限主问题、定价子问题
    node = Node()
    # 建立定价子问题模型,每一辆车都是一个定价子问题
    allSubset = network.allSubset
    CustomerLst = network.Customer
    sL = network.sL
    sR = network.sR
    sS = network.sS
    physicalCustomerList = network.physicalCustomer
    demand = network.demand
    vehicleCap = network.vehicleCap
    vehicleLst = network.vehicleLst
    vehicleNum = len(vehicleLst)

    for _index in range(0,len(network.vehicleLst)):
        # 建立定价子问题模型
        m = Model(f'{_index}_priceSP')
        # 定义变量：
        # Step1.添加决策变量/辅助变量
        '______添加决策变量X______'
        # 建立存储决策变量X的数据结构
        print('时空节点数量：', network.nodeNum)
        print('时间切片数量：', network.timeNum)
        print('状态数量：', network.stateNum)

        X = [[[[[[[] for _ in range(0, network.stateNum)] for _ in range(0, network.stateNum)] for _ in
                range(0, network.timeNum)] for _ in range(0, network.timeNum)] for _ in range(0, network.nodeNum)] for _ in
             range(0, network.nodeNum)]  # x_ijtsww'
        Y = [[[[[[[] for _ in range(0, network.stateNum)] for _ in range(0, network.stateNum)] for _ in
                range(0, network.timeNum)] for _ in range(0, network.timeNum)] for _ in range(0, network.nodeNum)] for _ in
             range(0, network.nodeNum)]  # y_ijtsww'
        D = [[[[[[[] for _ in range(0, network.timeNum)] for _ in range(0, network.timeNum)] for _ in
                range(0, network.timeNum)] for _ in range(0, network.nodeNum)] for _ in range(0, network.nodeNum)] for _ in
             range(0, network.nodeNum)]  # d_ijktsp
        P = [[] for _ in range(0, len(allSubset))]  # p_index 客户子集索引指示变量

        # 根据数据结构向模型中添加对应下标的变量
        for _ in network.stsVehicleArcList:
            i = _.from_space
            j = _.to_space
            t = _.from_time
            s = _.to_time
            w = _.from_state
            w_ = _.to_state
            # print(f"X_{i}_{j}_{t}_{s}_{w}_{w_}")
            X[i][j][t][s][w][w_] = m.addVar(lb=0, vtype=GRB.BINARY, name=f"X_{i}_{j}_{t}_{s}_{w}_{w_}")
        for _ in network.stsDroneArcList:
            i = _.from_space
            j = _.to_space
            t = _.from_time
            s = _.to_time
            w = _.from_state
            w_ = _.to_state
            Y[i][j][t][s][w][w_] = m.addVar(lb=0, vtype=GRB.BINARY, name=f"Y_{i}_{j}_{t}_{s}_{w}_{w_}")
        for _ in network.PotentialSortiesList:
            i = _.start_space
            j = _.delivery_space
            k = _.end_space
            t = _.start_time
            s = _.delivery_time
            p = _.end_time
            D[i][j][k][t][s][p] = m.addVar(lb=0,vtype=GRB.BINARY, name=f"D_{i}_{j}_{k}_{t}_{s}_{p}")

        for i_ in range(0,len(allSubset)):
            P[i_] = m.addVar(vtype=GRB.BINARY, name=f"P_{i_}")

        # 定义约束条件
        # 1.车辆空间-时间-状态起点流出约束（流入-流出=-1）
        num = 0
        for _ in network.stsNodeList:
            n = _.stsNode  # 时空状态节点 (n.space, n.time, n.state)
            k = n[0]
            p = n[1]
            l = n[2]
            if (k, p, l) == (network.O, 0, 0):  # (k,p,l)是起点
                expr = LinExpr(0)
                for flowInArc in network.stsVehicleArcList:
                    if (k == flowInArc.to_space) and (p == flowInArc.to_time) and (l == flowInArc.to_state):
                        i = flowInArc.from_space
                        t = flowInArc.from_time
                        w = flowInArc.from_state
                        expr.addTerms(1, X[i][k][t][p][w][l])
                for flowOutArc in network.stsVehicleArcList:
                    if (k == flowOutArc.from_space) and (p == flowOutArc.from_time) and (l == flowOutArc.from_state):
                        j = flowOutArc.to_space
                        s = flowOutArc.to_time
                        w_ = flowOutArc.to_state
                        expr.addTerms(-1, X[k][j][p][s][l][w_])
                num += 1
                m.addConstr(expr == -1, f'{_index}_C1_{num}')
        print("约束1已建立完成")

        # 2.车辆空间-时间-状态终点流入约束
        num = 0
        for _ in network.stsNodeList:
            n = _.stsNode  # 时空状态节点 (n.space, n.time, n.state)
            k = n[0]
            p = n[1]
            l = n[2]
            if (k, p, l) == (network.D, network.TTB, 0):  # (k,p,l)是终点
                expr = LinExpr(0)
                for flowInArc in network.stsVehicleArcList:
                    if (k == flowInArc.to_space) and (p == flowInArc.to_time) and (l == flowInArc.to_state):
                        i = flowInArc.from_space
                        t = flowInArc.from_time
                        w = flowInArc.from_state
                        expr.addTerms(1, X[i][k][t][p][w][l])
                for flowOutArc in network.stsVehicleArcList:
                    if (k == flowOutArc.from_space) and (p == flowOutArc.from_time) and (l == flowOutArc.from_state):
                        j = flowOutArc.to_space
                        s = flowOutArc.to_time
                        w_ = flowOutArc.to_state
                        expr.addTerms(-1, X[k][j][p][s][l][w_])
                num += 1
                m.addConstr(expr == 1, f'{_index}_C2_{num}')
        print("约束2已建立完成")

        # 3.车辆空间-时间-状态中间节点流平衡约束
        num = 0
        for _ in network.stsNodeList:
            n = _.stsNode  # 时空状态节点 (n.space, n.time, n.state)
            k = n[0]
            p = n[1]
            l = n[2]
            if ((k, p, l) == (network.O, 0, 0)) or ((k, p, l) == (network.D, network.TTB, 0)):  # (k,p,l)是除了时空状态起、终点之外的中间点
                pass
            else:
                expr = LinExpr(0)
                for flowInArc in network.stsVehicleArcList:
                    if (k == flowInArc.to_space) and (p == flowInArc.to_time) and (l == flowInArc.to_state):
                        i = flowInArc.from_space
                        t = flowInArc.from_time
                        w = flowInArc.from_state
                        expr.addTerms(1, X[i][k][t][p][w][l])
                for flowOutArc in network.stsVehicleArcList:
                    if (k == flowOutArc.from_space) and (p == flowOutArc.from_time) and (l == flowOutArc.from_state):
                        j = flowOutArc.to_space
                        s = flowOutArc.to_time
                        w_ = flowOutArc.to_state
                        expr.addTerms(-1, X[k][j][p][s][l][w_])
                num += 1
                m.addConstr(expr == 0, f'{_index}_C3_{num}_({k}_{p}_{l})')
        print("约束3已建立完成")

        # 4.无人机空间-时间-状态起点流出约束
        num = 0
        for _ in network.stsNodeList:
            n = _.stsNode  # 时空状态节点 (n.space, n.time, n.state)
            k = n[0]
            p = n[1]
            l = n[2]
            if (k, p, l) == (network.O, 0, 0):  # (k,p,l)是起点
                expr = LinExpr(0)
                for flowInArc in network.stsDroneArcList:
                    if (k == flowInArc.to_space) and (p == flowInArc.to_time) and (l == flowInArc.to_state):
                        i = flowInArc.from_space
                        t = flowInArc.from_time
                        w = flowInArc.from_state
                        expr.addTerms(1, Y[i][k][t][p][w][l])
                for flowOutArc in network.stsDroneArcList:
                    if (k == flowOutArc.from_space) and (p == flowOutArc.from_time) and (l == flowOutArc.from_state):
                        j = flowOutArc.to_space
                        s = flowOutArc.to_time
                        w_ = flowOutArc.to_state
                        expr.addTerms(-1, Y[k][j][p][s][l][w_])
                num += 1
                m.addConstr(expr == -1, f'{_index}_C4_{num}')
        print("约束4已建立完成")

        # 5.无人机空间-时间-状态终点流入约束
        num = 0
        for _ in network.stsNodeList:
            n = _.stsNode  # 时空状态节点 (n.space, n.time, n.state)
            k = n[0]
            p = n[1]
            l = n[2]
            if (k, p, l) == (network.D, network.TTB, 0):  # (k,p,l)是终点
                expr = LinExpr(0)
                for flowInArc in network.stsDroneArcList:
                    if (k == flowInArc.to_space) and (p == flowInArc.to_time) and (l == flowInArc.to_state):
                        i = flowInArc.from_space
                        t = flowInArc.from_time
                        w = flowInArc.from_state
                        expr.addTerms(1, Y[i][k][t][p][w][l])
                for flowOutArc in network.stsDroneArcList:
                    if (k == flowOutArc.from_space) and (p == flowOutArc.from_time) and (l == flowOutArc.from_state):
                        j = flowOutArc.to_space
                        s = flowOutArc.to_time
                        w_ = flowOutArc.to_state
                        expr.addTerms(-1, Y[k][j][p][s][l][w_])
                num += 1
                m.addConstr(expr == 1, f'{_index}_C5_{num}')
        print("约束5已建立完成")

        # 6.无人机空间-时间-状态中间节点流平衡约束
        num = 0
        for _ in network.stsNodeList:
            n = _.stsNode  # 时空状态节点 (n.space, n.time, n.state)
            k = n[0]
            p = n[1]
            l = n[2]
            if ((k, p, l) == (network.O, 0, 0)) or ((k, p, l) == (network.D, network.TTB, 0)):  # (k,p,l)是除了时空状态起、终点之外的中间点
                pass
            else:
                expr = LinExpr(0)
                for flowInArc in network.stsDroneArcList:
                    if (k == flowInArc.to_space) and (p == flowInArc.to_time) and (l == flowInArc.to_state):
                        i = flowInArc.from_space
                        t = flowInArc.from_time
                        w = flowInArc.from_state
                        expr.addTerms(1, Y[i][k][t][p][w][l])
                for flowOutArc in network.stsDroneArcList:
                    if (k == flowOutArc.from_space) and (p == flowOutArc.from_time) and (l == flowOutArc.from_state):
                        j = flowOutArc.to_space
                        s = flowOutArc.to_time
                        w_ = flowOutArc.to_state
                        expr.addTerms(-1, Y[k][j][p][s][l][w_])
                num += 1
                m.addConstr(expr == 0, f'{_index}_C6_{num}_({k}_{p}_{l})')
        print("约束6已建立完成")

        # 7.客户服务约束，服务弧来表示卡车
        num = 0
        for j_name in network.Customer:  # 虚拟客户点j
            j_index = network.Customer.index(j_name)  # 这个虚拟客户点的索引
            expr = LinExpr(0)
            for _ in network.stsVehicleArcList:
                i = _.from_space
                j = _.to_space
                t = _.from_time
                s = _.to_time
                w = _.from_state
                w_ = _.to_state
                tpye = _.arcType
                if (tpye == 4) and (j == j_name):
                    expr.addTerms(1, X[i][j][t][s][w][w_])  # 不满足时间窗的服务弧压根就不会构建

            for _ in network.PotentialSortiesList:
                i = _.start_space
                j = _.delivery_space  # 该架次的虚拟节点的索引
                k = _.end_space
                t = _.start_time
                s = _.delivery_time  # 该架次的虚拟节点的时间
                p = _.end_time
                if (j == j_name) and (s in network.TimeWindowMtx[j_index]):
                    expr.addTerms(1, D[i][j][k][t][s][p])
            num += 1
            m.addConstr(expr <= 1, f'{_index}_C9_{num}')  # 客户可被访问0或者1次
        print("约束7已建立完成")

        # 8.无人机飞行架次决策变量与无人机路由变量逻辑约束(1)
        num = 0
        for _ in network.PotentialSortiesList:
            i = _.start_space
            j = _.delivery_space  # 此处的j来自于架次，是虚拟的节点
            j_realistic = physicalCustomerList[CustomerLst.index(j)]  # 该虚拟节点对应的实际节点索引
            k = _.end_space
            t = _.start_time
            s = _.delivery_time
            p = _.end_time
            expr = LinExpr(0)
            expr.addTerms(1, Y[i][i][t - sL][t][0][1])
            expr.addTerms(1, Y[i][j_realistic][t][s - sS][1][1])
            expr.addTerms(1, Y[j_realistic][j][s - sS][s][1][1])
            expr.addTerms(1, Y[j][j_realistic][s][s + sS][1][1])
            expr.addTerms(1, Y[j_realistic][k][s + sS][p][1][1])
            expr.addTerms(1, Y[k][k][p][p + sR][1][0])
            expr.addTerms(-6, D[i][j][k][t][s][p])
            num += 1
            m.addConstr(expr >= 0, f'{_index}_C8_{num}_({t}_{s}_{p})')
        print("约束8已建立完成")

        # 9.无人机飞行架次决策变量与无人机路由变量逻辑约束(2)
        num = 0
        for _ in network.PotentialSortiesList:
            i = _.start_space
            j = _.delivery_space  # 此处的j来自于架次，是虚拟节点
            j_realistic = physicalCustomerList[CustomerLst.index(j)]  # 该虚拟节点对应的实际节点索引
            k = _.end_space
            t = _.start_time
            s = _.delivery_time
            p = _.end_time
            expr = LinExpr(0)
            expr.addTerms(1, Y[i][i][t - sL][t][0][1])
            expr.addTerms(1, Y[i][j_realistic][t][s - sS][1][1])
            expr.addTerms(1, Y[j_realistic][j][s - sS][s][1][1])
            expr.addTerms(1, Y[j][j_realistic][s][s + sS][1][1])
            expr.addTerms(1, Y[j_realistic][k][s + sS][p][1][1])
            expr.addTerms(1, Y[k][k][p][p + sR][1][0])
            expr.addTerms(-6, D[i][j][k][t][s][p])
            num += 1
            m.addConstr(expr <= 5, f'{_index}_C9_{num}_({t}_{s}_{p})')
        print("约束9已建立完成")

        # 10.卡车组容量约束
        expr = 0
        for j_name in network.Customer:  # 虚拟客户点j
            j_index = network.Customer.index(j_name)  # 这个虚拟客户点的索引
            demand_j = demand[j_index]  # 这个虚拟客户点的需求量
            expr1 = 0
            for _ in network.stsVehicleArcList:
                i = _.from_space
                j = _.to_space
                t = _.from_time
                s = _.to_time
                w = _.from_state
                w_ = _.to_state
                tpye = _.arcType
                if (tpye == 4) and (j == j_name):
                    expr1 += X[i][j][t][s][w][w_]  # 不满足时间窗的服务弧压根就不会构建

            expr2 = 0
            for _ in network.PotentialSortiesList:
                i = _.start_space
                j = _.delivery_space  # 该架次的虚拟节点的索引
                k = _.end_space
                t = _.start_time
                s = _.delivery_time  # 该架次的虚拟节点的时间
                p = _.end_time
                if (j == j_name) and (s in network.TimeWindowMtx[j_index]):
                    expr2 += D[i][j][k][t][s][p]
            expr += demand_j * (expr1 + expr2)
        m.addConstr(expr <= vehicleCap[_index], f'{_index}_C10_{num}')
        print("约束10已建立完成")

        # 11.车辆无人机路由合体约束
        num = 0
        for _ in network.stsVehicleArcRemovalList:
            i = _.from_space
            j = _.to_space
            t = _.from_time
            s = _.to_time
            w = _.from_state
            w_ = _.to_state
            expr = LinExpr(0)
            expr.addTerms(1, Y[i][j][t][s][w][w_])
            expr.addTerms(-1, X[i][j][t][s][w][w_])
            num += 1
            m.addConstr(expr == 0, f'{_index}_C10_{num}')
        print("约束11已建立完成")

        # 12.定义rounded capacity inequalities 辅助变量的取值【需要谨慎严格检查】
        num = 0
        for i_ in range(0, len(allSubset)):
            subC = allSubset[i_]  # 此时的C‘
            expr_sum = 0  # [release] sum of the service expressions of the customers of the subset
            for j_name in subC: # 客户name
                j_index = network.Customer.index(j_name) # 客户索引
                expr = 0
                for _ in network.stsVehicleArcList:
                    i = _.from_space
                    j = _.to_space
                    t = _.from_time
                    s = _.to_time
                    w = _.from_state
                    w_ = _.to_state
                    tpye = _.arcType
                    if (tpye == 4) and (j == j_name) and (w,w_)==(1,1):  # 卡车单独服务客户j_name的服务弧
                        expr += X[i][j][t][s][w][w_]  # 生成的服务弧全都是在时间窗里面的
                for _ in network.PotentialSortiesList:
                    i = _.start_space
                    j = _.delivery_space  # 该架次的虚拟节点的索引
                    k = _.end_space
                    t = _.start_time
                    s = _.delivery_time  # 该架次的虚拟节点的时间
                    p = _.end_time
                    if j == j_name:
                        expr += D[i][j][k][t][s][p]
                for _ in network.stsVehicleArcList:
                    i = _.from_space
                    j = _.to_space
                    t = _.from_time
                    s = _.to_time
                    w = _.from_state
                    w_ = _.to_state
                    tpye = _.arcType
                    if (tpye == 4) and (j == j_name) and (w,w_) == (0,0):  # 卡车和无人机一起服务客户j_name的服务弧
                        expr += X[i][j][t][s][w][w_]

                m.addConstr(P[i_] >= expr, f'{_index}_C11_{num}')
                num+=1
                expr_sum += expr  # [release]
            if UPPER_BOUND_ON_P:  # [release]
                m.addConstr(P[i_] <= expr_sum, f'{_index}_C11u_{i_}')
        print("约束12已建立完成")

        m.update()
        #m.write("check.lp")
        node.SP_lst.append(m)

    # 建立初始的受限主问题模型(线性规划问题)
    node.RMP = Model('ini_restrictedMP')
    Lambda = [[] for i in range(0, len(iniBase))]
    for i in range(0, len(iniBase)):
        Lambda[i] = node.RMP.addVar(lb=0, vtype=GRB.CONTINUOUS, name=f'{i}_Lambda_{0}')
        node.columnPool[f'{i}_Lambda_{0}'] = iniBase[i]  # 构建初始列池（这与初始基对应）

    node.columnPool_SP[f'{0}_Lambda_{0}'] = [
        f'X_{0}_{1}_{0}_{1}_{0}_{0}',
        f'X_{1}_{3}_{1}_{3}_{0}_{0}',
        f'X_{3}_{4}_{3}_{6}_{0}_{0}',
        f'X_{4}_{25}_{6}_{7}_{0}_{0}',
        f'X_{25}_{4}_{7}_{8}_{0}_{0}',
        f'X_{4}_{3}_{8}_{12}_{0}_{0}',
        f'X_{3}_{12}_{12}_{19}_{0}_{0}',
        f'X_{12}_{26}_{19}_{20}_{0}_{0}',
        f'X_{26}_{12}_{20}_{21}_{0}_{0}',
        f'X_{12}_{3}_{21}_{29}_{0}_{0}',
        f'X_{3}_{1}_{29}_{33}_{0}_{0}',
        f'X_{1}_{29}_{33}_{34}_{0}_{0}']

    node.columnPool_SP[f'{1}_Lambda_{0}'] = [
        f'X_{0}_{1}_{0}_{1}_{0}_{0}',
        f'X_{1}_{3}_{1}_{3}_{0}_{0}',
        f'X_{3}_{4}_{3}_{6}_{0}_{0}',
        f'X_{4}_{11}_{6}_{11}_{0}_{0}',
        f'X_{11}_{14}_{11}_{17}_{0}_{0}',
        f'X_{14}_{27}_{17}_{18}_{0}_{0}',
        f'X_{27}_{14}_{18}_{19}_{0}_{0}',
        f'X_{14}_{11}_{19}_{27}_{0}_{0}',
        f'X_{11}_{4}_{27}_{35}_{0}_{0}',
        f'X_{4}_{3}_{35}_{40}_{0}_{0}',
        f'X_{3}_{1}_{40}_{43}_{0}_{0}',
        f'X_{1}_{29}_{43}_{44}_{0}_{0}']

    node.columnPool_SP[f'{2}_Lambda_{0}'] = [
        f'X_{0}_{1}_{0}_{1}_{0}_{0}',
        f'X_{1}_{2}_{1}_{10}_{0}_{0}',
        f'X_{2}_{6}_{10}_{13}_{0}_{0}',
        f'X_{6}_{8}_{13}_{16}_{0}_{0}',
        f'X_{8}_{7}_{16}_{21}_{0}_{0}',
        f'X_{7}_{18}_{21}_{25}_{0}_{0}',
        f'X_{18}_{28}_{25}_{26}_{0}_{0}',
        f'X_{28}_{18}_{26}_{27}_{0}_{0}',
        f'X_{18}_{16}_{27}_{33}_{0}_{0}',
        f'X_{16}_{8}_{33}_{37}_{0}_{0}',
        f'X_{8}_{6}_{37}_{41}_{0}_{0}',
        f'X_{6}_{2}_{41}_{44}_{0}_{0}',
        f'X_{2}_{1}_{44}_{57}_{0}_{0}',
        f'X_{1}_{29}_{57}_{58}_{0}_{0}']
    node.RMP.update()

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

    # rounded capacity inequalities 先根据初始基，把框架搭起来，然后列生成向里面添加列
    for i in range(0, len(allSubset)):
        expr = 0
        subSet = allSubset[i] # 客户集合的子集 [10], [11], [12], [13], [10, 11]...
        name_indexLst = []
        for name in subSet:
            name_index = CustomerLst.index(name)  # 确定客户对应的索引
            name_indexLst.append(name_index)  # 如客户子集的索引列表[0,1]
        # print("此时客户索引列表是",name_indexLst)

        for vehicle_index in range(0, vehicleNum):  # 两个子问题
            # 分别找出两个子问题下面的初始列里面，哪些经过了客户子集C'中的客户，但凡经过一个，就将初始列加到约束中
            base = iniBase[vehicle_index][:len(network.Customer)]  # 当前子问题对应的列
            flag = 0
            for customerIndex in name_indexLst:  # 当前客户子集C’对应的客户索引
                if base[customerIndex] == 1:  # 说明此时的base经过了客户子集C’中索引为customerIndex的客户
                    flag=1
            if flag==1:
                expr += node.RMP.getVarByName(f'{vehicle_index}_Lambda_{0}') # 当前车辆的基（路径）iniBase = [[1, 0, 0, 1, 1, 0], [0, 1, 1, 0, 0, 1]]
            # print("当前客户子集下，添加的变量为",f'{vehicle_index}_Lambda_{0}')
        RHS = RHSnum(demand,CustomerLst,subSet,vehicleCap)  # 约束右端项
        # print("当前客户子集下，添加的RHS为", RHS)
        # print(f"最终在{subSet}的客户名字下，添加的约束是{expr >= RHS}")
        node.RMP.addConstr(expr >= RHS, f'roundedCons_{i}')

    #node.RMP.write('RMP_VRPDSTS.lp')

    return node


def columnGeneration(node, network,timeLimitation):  # 输出完备的RMP
    node.RMP.setParam("OutputFlag", 0)
    node.RMP.optimize()
    # 进入CG循环
    # print("RMP.Status", RMP.Status)
    SPtotalTimeCost = 0
    RMPtotalTimeCost = 0

    if node.RMP.Status != GRB.OPTIMAL:  # 由于行的加入，主问题可能无解
        print("——————该RMP不可行，割掉当前节点——————")
        return node,SPtotalTimeCost,RMPtotalTimeCost

    while (node.RMP.Status == GRB.OPTIMAL):

        for var in node.RMP.getVars():  # 对于RMP解中的任意一个变量
            # print(var.VarName, '=', var.x)
            node.Lambda[var.VarName] = var.x
        print('这个节点在列生成前，主问题变量node.Lambda有:', node.Lambda)

        sumRC = 0  # 初始检验数之和

        for _index in range(0, len(node.SP_lst)):
            SP_i = node.SP_lst[_index]  # 第_index个子问题
            SP_i.setParam("OutputFlag", 0)
            SP_i.setParam('Threads', 1)  # 单线程

            # 根据主问题变量数量，结合当前子问题情况，来设置生成列的命名起点：（1）先找到属于这一个子问题的列
            prefix = f"{_index}"
            filtered_keys = [key for key in node.Lambda.keys() if key.startswith(prefix)]  # 筛选出这个子问题i下面的所有相关的lambda
            # print('filtered_keys',filtered_keys)
            if len(filtered_keys) == 0:
                Iter = 0
            else:
                a = filtered_keys[-1].split('_')
                print(a)
                if a[1] != 'newLambda':
                    Iter = 0
                elif a[1] == 'newLambda':
                    Iter = int(a[2])

            # 更新子问题目标函数
            expr1 = 0
            for _ in network.stsVehicleArcList:
                i = _.from_space
                j = _.to_space
                t = _.from_time
                s = _.to_time
                w = _.from_state
                w_ = _.to_state
                arcType = _.arcType  # 0-运输弧、1-等待弧、2-发射弧、3-回收弧、4-服务弧
                arcCost = _.cost * 1  # 1的话那就等于时间了
                expr1 += arcCost * SP_i.getVarByName(f'X_{i}_{j}_{t}_{s}_{w}_{w_}')

            expr2 = 0
            for custIndex in range(0, len(network.Customer)):
                customer_name = network.Customer[custIndex]
                multiper_j = node.RMP.getConstrByName(f'C_{custIndex}').pi
                sum_expr = 0
                for _ in network.stsVehicleArcList:
                    i = _.from_space
                    j = _.to_space
                    t = _.from_time
                    s = _.to_time
                    w = _.from_state
                    w_ = _.to_state
                    arcType = _.arcType  # 0-运输弧、1-等待弧、2-发射弧、3-回收弧、4-服务弧
                    if arcType == 4 and j == customer_name and (w, w_) == (1, 1):
                        sum_expr += SP_i.getVarByName(f'X_{i}_{j}_{t}_{s}_{w}_{w_}')

                for _ in network.PotentialSortiesList:
                    i = _.start_space
                    j = _.delivery_space
                    k = _.end_space
                    t = _.start_time
                    s = _.delivery_time
                    p = _.end_time
                    if j == customer_name:
                        sum_expr += SP_i.getVarByName(f'D_{i}_{j}_{k}_{t}_{s}_{p}')

                for _ in network.stsVehicleArcList:
                    i = _.from_space
                    j = _.to_space
                    t = _.from_time
                    s = _.to_time
                    w = _.from_state
                    w_ = _.to_state
                    arcType = _.arcType  # 0-运输弧、1-等待弧、2-发射弧、3-回收弧、4-服务弧
                    if arcType == 4 and j == customer_name and (w, w_) == (0, 0):
                        sum_expr += SP_i.getVarByName(f'X_{i}_{j}_{t}_{s}_{w}_{w_}')
                expr2 += sum_expr * multiper_j
            expr3 = node.RMP.getConstrByName(f'C_{len(network.Customer)+_index}').pi

            expr4 = 0  # 这个用来存放roundedCons约束的乘子
            for subIndex in range(0,len(network.allSubset)):
                # print(f"p_i的乘子为：{node.RMP.getConstrByName(f'roundedCons_{subIndex}').pi}")
                expr4 += SP_i.getVarByName(f'P_{subIndex}')*(node.RMP.getConstrByName(f'roundedCons_{subIndex}').pi)
            expr5 = 0  # 这个是针对分数lambda后来添加的约束，是否要纳入RC的计算？——————是，但由于都是0，所以无需添加

            expr_punish = 0
            for subIndex in range(0,len(network.allSubset)):
                expr_punish += SP_i.getVarByName(f'P_{subIndex}')*1e-8
                # 此处加1e-8，是为了保证在pi取0的时候，P变量的优化的方向是有限取0而不是1（保证优化方向性），且1e-8小于最终对于sumRC在1e-4的判定标准

            SP_i.setObjective(expr1 - expr2 - expr3- expr4 - expr5 + expr_punish, sense=GRB.MINIMIZE)
            SPstartTime = time.time()  # 子问题求解开始时间
            SP_i.optimize()
            SPendTime = time.time()  # 子问题求解结束时间
            SPtotalTimeCost += SPendTime - SPstartTime

            if SP_i.Status != GRB.OPTIMAL:
                raise ("问题有误")
            else:
                sumRC += round(SP_i.ObjVal, 4)  # 子问题的检验数之和

            if _index == len(node.SP_lst) - 1 and sumRC >= 0:  # 对于最后一个子问题，如果到现在的sum都是0（说明这个最后一个子问题的值也是0），则1.不需要进行列生成，跳出 for 循环（最后一个位置continue也可以实现），2.跳出while循环
                break

            if round(SP_i.ObjVal, 4) >= 0:  # 任何一个检验数大于等于0的子问题，都不需要进行列生成，而是直接跳到下一个
                continue

            Iter += 1
            print(f"Iter:{Iter},RMP.ObjVal:{round(node.RMP.ObjVal)},SP_i.ObjVal:{round(SP_i.ObjVal)}")

            # 构建列对象并将新列加入RMP
            col = Column()  # 新建列对象，为了有效地进行列管理
            col_lst = []
            col_lst_SP = []

            # 生成列
            for custIndex in range(0, len(network.Customer)):
                customer_name = network.Customer[custIndex]
                alpha_i = 0
                for _ in network.stsVehicleArcList:
                    i = _.from_space
                    j = _.to_space
                    t = _.from_time
                    s = _.to_time
                    w = _.from_state
                    w_ = _.to_state
                    arcType = _.arcType  # 0-运输弧、1-等待弧、2-发射弧、3-回收弧、4-服务弧
                    if arcType == 4 and j == customer_name and (w,w_)==(1,1):
                        alpha_i += round(SP_i.getVarByName(f'X_{i}_{j}_{t}_{s}_{w}_{w_}').X)

                beta_i = 0
                for _ in network.PotentialSortiesList:
                    i = _.start_space
                    j = _.delivery_space
                    k = _.end_space
                    t = _.start_time
                    s = _.delivery_time
                    p = _.end_time
                    if j == customer_name:
                        beta_i += round(SP_i.getVarByName(f'D_{i}_{j}_{k}_{t}_{s}_{p}').X)

                gama_i = 0
                for _ in network.stsVehicleArcList:
                    i = _.from_space
                    j = _.to_space
                    t = _.from_time
                    s = _.to_time
                    w = _.from_state
                    w_ = _.to_state
                    arcType = _.arcType  # 0-运输弧、1-等待弧、2-发射弧、3-回收弧、4-服务弧
                    if arcType == 4 and j == customer_name and (w,w_)== (0,0):
                        gama_i += round(SP_i.getVarByName(f'X_{i}_{j}_{t}_{s}_{w}_{w_}').X)

                coeff = alpha_i+beta_i+gama_i

                col_lst.append((alpha_i,beta_i,gama_i))
                col.addTerms(coeff, node.RMP.getConstrByName(f'C_{custIndex}'))

            for i in range(0,len(network.vehicleLst)):  # 车辆路径数量约束
                if i == _index:  # 如果对应的约束就是应当施加在这个子问题（车辆）上面的
                    col.addTerms(1, node.RMP.getConstrByName(f'C_{len(network.Customer)+i}'))  # 异质车辆路径数量约束
                    col_lst.append(1)
                else:
                    col.addTerms(0, node.RMP.getConstrByName(f'C_{len(network.Customer)+i}'))  # 异质车辆路径数量约束
                    col_lst.append(0)

            for i in range(0,len(network.allSubset)):  # 对于所有的客户节点的集合C‘
                col.addTerms(round(SP_i.getVarByName(f'P_{i}').X), node.RMP.getConstrByName(f'roundedCons_{i}'))
                col_lst.append(round(SP_i.getVarByName(f'P_{i}').X))

            """_____________________________开始统计经过的路段_____________________________"""
            for _ in network.stsVehicleArcList:  # 生成这个列对应的路由变量情况，这个只跟变量x有关
                i = _.from_space
                j = _.to_space
                t = _.from_time
                s = _.to_time
                w = _.from_state
                w_ = _.to_state

                if round(SP_i.getVarByName(f'X_{i}_{j}_{t}_{s}_{w}_{w_}').X, 4) == 1:
                    col_lst_SP.append(SP_i.getVarByName(f'X_{i}_{j}_{t}_{s}_{w}_{w_}').VarName)

            for _ in network.PotentialSortiesList:
                i = _.start_space
                j = _.delivery_space
                k = _.end_space
                t = _.start_time
                s = _.delivery_time
                p = _.end_time
                if round(SP_i.getVarByName(f'D_{i}_{j}_{k}_{t}_{s}_{p}').X, 4) == 1:  # 输入增加的无人机路由相关的六条弧？
                    col_lst_SP.append(SP_i.getVarByName(f'D_{i}_{j}_{k}_{t}_{s}_{p}').VarName)

            print("生成的列", col_lst)
            print('生成列的路由详情', col_lst_SP)

            cp = 0  # 此列对应的成本
            for _ in network.stsVehicleArcList:
                i = _.from_space
                j = _.to_space
                t = _.from_time
                s = _.to_time
                w = _.from_state
                w_ = _.to_state
                arcType = _.arcType  # 0-运输弧、1-等待弧、2-发射弧、3-回收弧、4-服务弧
                arcCost = _.cost * 1  # 1的话那就等于时间了
                cp += arcCost * SP_i.getVarByName(f'X_{i}_{j}_{t}_{s}_{w}_{w_}').X  # 在目标函数中，某条弧对应的车辆的成本，即为车辆在这条弧上花费的时间

            node.RMP.addVar(obj=cp, column=col, lb=0, ub=1, name=f"{_index}_newLambda_" + str(Iter))
            node.columnPool[f"{_index}_newLambda_" + str(Iter)] = col_lst  # 随着列生成不断丰富列池
            node.columnPool_SP[f"{_index}_newLambda_" + str(Iter)] = col_lst_SP  # 随着列生成不断丰富列池

        print(f'总Gap为：{-sumRC}')
        if sumRC >= 0 or SPtotalTimeCost > timeLimitation:  # 子问题的检验数之和等于0
            break

        node.RMP.update()
        RMPstartTime = time.time()  # 主问题求解开始时间
        node.RMP.optimize()
        RMPendTime = time.time()  # 主问题求解结束时间
        RMPtotalTimeCost += RMPendTime - RMPstartTime

        print("node.RMP的求解状态是：",node.RMP.Status)

    for var in node.RMP.getVars():  # 对于RMP解中的任意一个变量
        node.Lambda[var.VarName] = var.x
    print('这个节点在列生成后，主问题变量node.Lambda有:',node.Lambda)
    # node.RMP.write(f"final_RMP_VRPDSTS.lp")

    return node,SPtotalTimeCost,RMPtotalTimeCost


def branchAndPrice(iniNode,eps,globalUB,globalLB, network,timeLimitation):  # 初始节点

    nodeLst = [iniNode]  # BB tree node list
    incumbent = None  # 最优解
    global_UB_lst = [globalUB]
    global_LB_lst = [globalLB]

    SPtimeCostInBP = 0  # 分支定价中的子问题总求解时间
    RMPtimeCostInBP = 0  # 分支定价中的主问题总求解时间

    while globalUB-globalLB > eps:
        print('分支树中的nodeLst', nodeLst)
        currentNode = nodeLst.pop()
        print('所选择的currentNode', currentNode)

        '''for _index in range(0,len(currentNode.SP_lst)): # 此处后续可省略用于加速
            SP_i = currentNode.SP_lst[_index]
            SP_i.setParam("OutputFlag", 0)
            SP_i.optimize()
            print(_index,SP_i.Status)
            if SP_i.Status != GRB.OPTIMAL:
                SP_i.write('check.lp')
                raise ("问题有误")'''

        currentNode,SPtimeCost,RMPtimeCost = columnGeneration(currentNode, network,timeLimitation)
        SPtimeCostInBP += SPtimeCost
        RMPtimeCostInBP += RMPtimeCost

        if SPtimeCostInBP > timeLimitation:
            return incumbent,global_UB_lst,global_LB_lst,SPtimeCostInBP,RMPtimeCostInBP

        is_integer = True  # 后续只要有一个分量不是整数，都会使其变为False，说明当前这个节点（问题）不是整数解
        is_Pruned = False  # 后续只要有一个符合剪枝条件，都会使其变为True，说明当前这个节点（问题）被剪枝了

        if currentNode.RMP.Status == GRB.OPTIMAL:  # 对于当前节点的RMP，如果求得最优解
            # print('列生成后，当前节点的主问题变量currentNode.Lambda:',currentNode.Lambda)

            for var in currentNode.RMP.getVars():  # 对于解中的任意一个变量
                # print(var.VarName, '=', var.x)
                currentNode.Lambda[var.VarName] = var.x
                if abs(var.x - int(var.x)) > eps:  # 说明不是整数解，将这一个解放入候选分支变量集合中
                    is_integer = False
                    currentNode.branch_var_lst.append(var.VarName)
                else:
                    currentNode.int_lambda[var.VarName] = int(var.x)
            print("非整数解为(待分支变量为)：",currentNode.branch_var_lst)
            valueRMP = currentNode.RMP.ObjVal  # 列生成之后的最终主问题的目标函数值（松弛目标函数值）
            totalRC = 0
            for _index in range(0, len(currentNode.SP_lst)):
                totalRC += currentNode.SP_lst[_index].ObjVal
            currentNode.reducedCost = totalRC  # 最终所有子问题的目标函数值(检验数)
            print("列生成后RMP函数值,所有SP检验数之和", valueRMP, round(currentNode.reducedCost, 4))

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
            # 根据最不可行分支规则，确定分支变量
            branchVarName = currentNode.branch_var_lst[0]
            distance_05 = abs(currentNode.Lambda[branchVarName] - int(currentNode.Lambda[branchVarName]) - 0.5)
            for varName in currentNode.branch_var_lst:
                distance_var = abs(currentNode.Lambda[varName] - int(currentNode.Lambda[varName])-0.5)
                if distance_var < distance_05:
                    branchVarName = varName
                    distance_05 = distance_var

            print("根据最不可行分支规则，确定分支变量为", branchVarName)
            # 根据分支，创建子节点
            leftNode = Node.deepcopy(currentNode)  # 剔除这个基
            rightNode = Node.deepcopy(currentNode)  # 保留这个基

            # 确定分支变量对应的子问题
            a = branchVarName.split('_')  # branchVarName 是 f'{i}_Lambda_{0}'形式
            SP_index = int(a[0])  # 第几个子问题

            # 确定分支变量对应的基
            currentBase = currentNode.columnPool[branchVarName]  # 用于RMP
            currentBase_SP = currentNode.columnPool_SP[branchVarName]  # 用于SP
            print('分支变量对应的基', currentBase)
            print('分支变量对应的基所对应的子问题解', currentBase_SP)

            """ 更新对应子节点的RMP,SP——leftNode：剔除这个基（基于路径） """
            # 更新左节点的RMP(将该基的使用系数设置为0)
            targetVar = leftNode.RMP.getVarByName(branchVarName)
            print("left RMP:targetVar", targetVar)
            leftNode.RMP.addConstr(targetVar == 0, f'NOT_RMPcons_'+branchVarName)  # 在主问题中使得这个基的使用系数（主问题变量=0）
            leftNode.RMPcons.append(f'NOT_RMPcons_'+branchVarName)
            leftNode.RMP.setParam('OutputFlag', 0)
            leftNode.RMP.update()
            # 更新左节点的SP（将该路径剔除/破圈约束）:对branchVarName对应的子问题添加约束
            expr = 0
            for varName in currentBase_SP:
                expr += leftNode.SP_lst[SP_index].getVarByName(varName)
            # print('expr', expr)
            print('路径的弧数量',len(currentBase_SP))
            leftNode.SP_lst[SP_index].addConstr(expr <= len(currentBase_SP)-1, f'NOT_SPcons_'+branchVarName)
            leftNode.SP_lst[SP_index].setParam('OutputFlag', 0)
            leftNode.SP_lst[SP_index].update()

            """ 更新对应子节点的RMP,SP——rightNode：选中这个基 """
            # 更新右节点RMP(将该基系数设置为1)
            targetVar = rightNode.RMP.getVarByName(branchVarName)
            print("RMP中需要固定为1的分支变量为", targetVar)
            rightNode.RMP.addConstr(targetVar == 1, f'YES_RMPcons_'+branchVarName)  # 在主问题中使得这个基的使用系数=1
            rightNode.RMPcons.append(f'YES_RMPcons_'+branchVarName)
            rightNode.RMP.setParam('OutputFlag', 0)
            rightNode.RMP.update()
            # 让这几个x等于1即可固定（代理约束形式）
            expr = 0
            for varName in currentBase_SP:
                expr += rightNode.SP_lst[SP_index].getVarByName(varName)
            rightNode.SP_lst[SP_index].addConstr(expr == len(currentBase_SP), name=f'YES_SPCons_'+branchVarName)
            # print('expr', expr)
            print('路径的弧数量', len(currentBase_SP))
            rightNode.SP_lst[SP_index].setParam('OutputFlag', 0)
            rightNode.SP_lst[SP_index].update()

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

    return incumbent,global_UB_lst,global_LB_lst,SPtimeCostInBP,RMPtimeCostInBP


if __name__ == "__main__":
    eps = 0.0001
    timeLimitation = 7200
    globalUB, globalLB = np.inf, 0
    CustomerLst = [25, 26, 27, 28]  # 虚拟客户点id
    physicalCustomerList = [4, 12, 14, 18]  # 实际的客户点id
    demand = [4,7,3,5]  # 客户需求量
    O = 0
    D = 29
    Vehicle_Cap = [55, 55, 55]  # 卡车组容量
    Truckcapacity = [50, 50, 50]  # 卡车容量
    UAVcapacity = [5, 5, 5]  # 无人机容量
    UAVendurance = [15, 15, 15]  # 无人机续航
    vehicleNum = 3  # 车辆数量
    iniBase = [[1, 1, 0, 0, 1, 0, 0], [0, 0, 1, 0, 0, 1, 0], [0, 0, 0, 1, 0, 0, 1]]  # 长度=C个客户+v（车辆数）条约束
    iniCost = [34, 44, 58]

    vehicleLst = [i for i in range(0, vehicleNum)]
    sL = 1
    sR = 1
    sS = 1

    st_network = TDconstruct4.network
    st_network.readStsNode(physicalCustomerList, CustomerLst, vehicleLst, demand, Vehicle_Cap,Truckcapacity,UAVcapacity, UAVendurance, sL, sR, sS, O, D)
    st_network.enumerate_subsets()  # [release] the network module no longer enumerates the customer subsets (lazy RCI separation in the B&P drivers); this script adds the inequalities a priori and needs them
    st_network.readStsVehicleArc()
    st_network.readPotentialSorties(sL, sR)
    st_network.readStsDroneArc()

    print("————————分支定价—————————")
    iniNode = modeling(st_network, iniBase, iniCost)
    start_time = time.time()
    optimalIntegerSol,global_UB_lst,global_LB_lst,SPtimeCostInBP,RMPtimeCostInBP = branchAndPrice(iniNode,eps,globalUB,globalLB, st_network,timeLimitation)
    BP_total = time.time() - start_time
    print(f"分支定价求解时间: {BP_total} s")
    print(f"分支定价中子问题总求解时间: {SPtimeCostInBP} s")
    print(f"分支定价中主问题总求解时间: {RMPtimeCostInBP} s")

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
    instanceName = 'timeDependent_SFnetwork_4customer'
    result_Path = fr'{RESULT_DIR}/Performance of acceleration components/{instanceName}/result(SP_gurobi).txt'
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
