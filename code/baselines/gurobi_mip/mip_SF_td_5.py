# -*- coding: utf-8 -*-
# Gurobi MIP of the arc-based VRPD-STS formulation (Section 4) for instance timeDependent_SFnetwork_5customer (Table 10, "Gurobi").
# Original file name in the authors' project: 时变-实例计算-SF5节点(0.用gurobi求解).py (last edited 2025/5/16)
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
from gurobipy import *
import TDconstructStsNetwork5 as TDconstruct5
import time


"""——————————————最短路上时间问题，起点和终点的等待弧段费用设置为0,其他为s-t,已经在弧文件中设置OK——————————————"""
'''建模和求解。使用Gurobi对问题进行建模'''
def modelingAndSolve(network):
    sL = network.sL
    sR = network.sR
    sS = network.sS
    physicalCustomerList = network.physicalCustomer
    CustomerLst = network.Customer
    demand = network.demand
    vehicleCap = network.vehicleCap
    vehicleLst = network.vehicleLst
    vehicleNum = len(vehicleLst)

    # 建立模型
    m = Model('SpaceTimeState_FSTSP')

    # 模型设置：由于存在函数printSolution，因此关闭输出;以及容许误差
    # m.setParam('OutputFlag', 0)

    # 定义变量：
    # Step1.添加决策变量/辅助变量
    '______添加决策变量X______'
    # 建立存储决策变量X的数据结构
    print('时空节点数量：',network.nodeNum)
    print('时间切片数量：',network.timeNum)
    print('状态数量：',network.stateNum)

    X = [[[[[[[[] for _ in range(0, network.stateNum)] for _ in range(0, network.stateNum)] for _ in
            range(0, network.timeNum)] for _ in range(0, network.timeNum)] for _ in range(0, network.nodeNum)] for _ in
         range(0, network.nodeNum)] for _ in range(0, vehicleNum)]  # x_ijtsww'_v
    Y = [[[[[[[[] for _ in range(0, network.stateNum)] for _ in range(0, network.stateNum)] for _ in
            range(0, network.timeNum)] for _ in range(0, network.timeNum)] for _ in range(0, network.nodeNum)] for _ in
         range(0, network.nodeNum)] for _ in range(0, vehicleNum)]  # y_ijtsww'_v
    D = [[[[[[[[] for _ in range(0, network.timeNum)] for _ in range(0, network.timeNum)] for _ in
            range(0, network.timeNum)] for _ in range(0, network.nodeNum)] for _ in range(0, network.nodeNum)] for _ in
         range(0, network.nodeNum)] for _ in range(0, vehicleNum)]  # d_ijktsp_v


    print("正在建立变量")
    # 根据数据结构向模型中添加对应下标的变量
    for v in vehicleLst:
        for _ in network.stsVehicleArcList:
            i = _.from_space
            j = _.to_space
            t = _.from_time
            s = _.to_time
            w = _.from_state
            w_ = _.to_state
            X[v][i][j][t][s][w][w_] = m.addVar(lb=0,vtype=GRB.BINARY, name=f"X_{v}_{i}_{j}_{t}_{s}_{w}_{w_}")
        for _ in network.stsDroneArcList:
            i = _.from_space
            j = _.to_space
            t = _.from_time
            s = _.to_time
            w = _.from_state
            w_ = _.to_state
            Y[v][i][j][t][s][w][w_] = m.addVar(lb=0,vtype=GRB.BINARY, name=f"Y_{v}_{i}_{j}_{t}_{s}_{w}_{w_}")
        for _ in network.PotentialSortiesList:
            i = _.start_space
            j = _.delivery_space
            k = _.end_space
            t = _.start_time
            s = _.delivery_time
            p = _.end_time
            D[v][i][j][k][t][s][p] = m.addVar(vtype=GRB.BINARY, name=f"D_{v}_{i}_{j}_{k}_{t}_{s}_{p}")


    m.update()
    var_lst = m.getVars()
    # print("变量总数是",len(var_lst))

    # 定义目标函数
    obj = LinExpr(0)  # 线性项，初始值为0，可用.addTerms(a,x) 意为将变量x增加到表达式，系数为a
    for v in vehicleLst:
        for _ in network.stsVehicleArcList:
            i = _.from_space
            j = _.to_space
            t = _.from_time
            s = _.to_time
            w = _.from_state
            w_ = _.to_state
            arcType = _.arcType  # 0-运输弧、1-等待弧、2-发射弧、3-回收弧、4-服务弧
            arcCost = _.cost * 1  # 1的话那就等于时间了
            obj.addTerms(arcCost, X[v][i][j][t][s][w][w_])
    m.setObjective(obj, sense=GRB.MINIMIZE)
    print("目标函数定义完成")

    # 定义约束条件

    # 3.车辆空间-时间-状态起点流出约束（流入-流出=-1）
    num = 0
    for v in vehicleLst:
        for _ in network.stsNodeList:
            node = _.stsNode  # 时空状态节点 (n.space, n.time, n.state)
            k = node[0]
            p = node[1]
            l = node[2]
            if (k,p,l) == (network.O,0,0):  # (k,p,l)是起点
                expr = LinExpr(0)
                for flowInArc in network.stsVehicleArcList:
                    if (k == flowInArc.to_space) and (p == flowInArc.to_time) and (l == flowInArc.to_state):
                        i = flowInArc.from_space
                        t = flowInArc.from_time
                        w = flowInArc.from_state
                        expr.addTerms(1, X[v][i][k][t][p][w][l])
                for flowOutArc in network.stsVehicleArcList:
                    if (k == flowOutArc.from_space) and (p == flowOutArc.from_time) and (l == flowOutArc.from_state):
                        j = flowOutArc.to_space
                        s = flowOutArc.to_time
                        w_ = flowOutArc.to_state
                        expr.addTerms(-1, X[v][k][j][p][s][l][w_])
                num += 1
                m.addConstr(expr == -1, f'C3_{num}')
    print("约束3已建立完成")

    # 4.车辆空间-时间-状态终点流入约束
    num = 0
    for v in vehicleLst:
        for _ in network.stsNodeList:
            node = _.stsNode  # 时空状态节点 (n.space, n.time, n.state)
            k = node[0]
            p = node[1]
            l = node[2]
            if (k,p,l) == (network.D,network.TTB,0):  # (k,p,l)是终点
                expr = LinExpr(0)
                for flowInArc in network.stsVehicleArcList:
                    if (k == flowInArc.to_space) and (p == flowInArc.to_time) and (l == flowInArc.to_state):
                        i = flowInArc.from_space
                        t = flowInArc.from_time
                        w = flowInArc.from_state
                        expr.addTerms(1, X[v][i][k][t][p][w][l])
                for flowOutArc in network.stsVehicleArcList:
                    if (k == flowOutArc.from_space) and (p == flowOutArc.from_time) and (l == flowOutArc.from_state):
                        j = flowOutArc.to_space
                        s = flowOutArc.to_time
                        w_ = flowOutArc.to_state
                        expr.addTerms(-1, X[v][k][j][p][s][l][w_])
                num += 1
                m.addConstr(expr == 1, f'C4_{num}')
    print("约束4已建立完成")

    # 5.车辆空间-时间-状态中间节点流平衡约束
    num = 0
    for v in vehicleLst:
        for _ in network.stsNodeList:
            node = _.stsNode  # 时空状态节点 (n.space, n.time, n.state)
            k = node[0]
            p = node[1]
            l = node[2]
            if ((k,p,l) == (network.O,0,0)) or ((k,p,l) == (network.D,network.TTB,0)):  # (k,p,l)是除了时空状态起、终点之外的中间点
                pass
            else:
                expr = LinExpr(0)
                for flowInArc in network.stsVehicleArcList:
                    if (k == flowInArc.to_space) and (p==flowInArc.to_time) and (l==flowInArc.to_state):
                        i = flowInArc.from_space
                        t = flowInArc.from_time
                        w = flowInArc.from_state
                        expr.addTerms(1,X[v][i][k][t][p][w][l])
                for flowOutArc in network.stsVehicleArcList:
                    if (k == flowOutArc.from_space) and (p==flowOutArc.from_time) and (l==flowOutArc.from_state):
                        j = flowOutArc.to_space
                        s = flowOutArc.to_time
                        w_ = flowOutArc.to_state
                        expr.addTerms(-1,X[v][k][j][p][s][l][w_])
                num += 1
                m.addConstr(expr == 0, f'C5_{num}_({k}_{p}_{l})')
    print("约束5已建立完成")

    # 6.无人机空间-时间-状态起点流出约束
    num = 0
    for v in vehicleLst:
        for _ in network.stsNodeList:
            node = _.stsNode  # 时空状态节点 (n.space, n.time, n.state)
            k = node[0]
            p = node[1]
            l = node[2]
            if (k,p,l) == (network.O,0,0):  # (k,p,l)是起点
                expr = LinExpr(0)
                for flowInArc in network.stsDroneArcList:
                    if (k == flowInArc.to_space) and (p == flowInArc.to_time) and (l == flowInArc.to_state):
                        i = flowInArc.from_space
                        t = flowInArc.from_time
                        w = flowInArc.from_state
                        expr.addTerms(1, Y[v][i][k][t][p][w][l])
                for flowOutArc in network.stsDroneArcList:
                    if (k == flowOutArc.from_space) and (p == flowOutArc.from_time) and (l == flowOutArc.from_state):
                        j = flowOutArc.to_space
                        s = flowOutArc.to_time
                        w_ = flowOutArc.to_state
                        expr.addTerms(-1, Y[v][k][j][p][s][l][w_])
                num += 1
                m.addConstr(expr == -1, f'C6_{num}')
    print("约束6已建立完成")

    # 7.无人机空间-时间-状态终点流入约束
    num = 0
    for v in vehicleLst:
        for _ in network.stsNodeList:
            node = _.stsNode  # 时空状态节点 (n.space, n.time, n.state)
            k = node[0]
            p = node[1]
            l = node[2]
            if (k,p,l) == (network.D,network.TTB,0): # (k,p,l)是终点
                expr = LinExpr(0)
                for flowInArc in network.stsDroneArcList:
                    if (k == flowInArc.to_space) and (p == flowInArc.to_time) and (l == flowInArc.to_state):
                        i = flowInArc.from_space
                        t = flowInArc.from_time
                        w = flowInArc.from_state
                        expr.addTerms(1, Y[v][i][k][t][p][w][l])
                for flowOutArc in network.stsDroneArcList:
                    if (k == flowOutArc.from_space) and (p == flowOutArc.from_time) and (l == flowOutArc.from_state):
                        j = flowOutArc.to_space
                        s = flowOutArc.to_time
                        w_ = flowOutArc.to_state
                        expr.addTerms(-1, Y[v][k][j][p][s][l][w_])
                num += 1
                m.addConstr(expr == 1, f'C7_{num}')
    print("约束7已建立完成")

    # 8.无人机空间-时间-状态中间节点流平衡约束
    num = 0
    for v in vehicleLst:
        for _ in network.stsNodeList:
            node = _.stsNode  # 时空状态节点 (n.space, n.time, n.state)
            k = node[0]
            p = node[1]
            l = node[2]
            if ((k, p, l) == (network.O, 0, 0)) or ((k, p, l) == (network.D, network.TTB, 0)):  # (k,p,l)是除了时空状态起、终点之外的中间点
                pass
            else:
                expr = LinExpr(0)
                for flowInArc in network.stsDroneArcList:
                    if (k == flowInArc.to_space) and (p == flowInArc.to_time) and (l == flowInArc.to_state):
                        i = flowInArc.from_space
                        t = flowInArc.from_time
                        w = flowInArc.from_state
                        expr.addTerms(1, Y[v][i][k][t][p][w][l])
                for flowOutArc in network.stsDroneArcList:
                    if (k == flowOutArc.from_space) and (p == flowOutArc.from_time) and (l == flowOutArc.from_state):
                        j = flowOutArc.to_space
                        s = flowOutArc.to_time
                        w_ = flowOutArc.to_state
                        expr.addTerms(-1, Y[v][k][j][p][s][l][w_])
                num += 1
                m.addConstr(expr == 0, f'C8_{num}_({k}_{p}_{l})')
    print("约束8已建立完成")

    # 9.客户服务约束，服务弧来表示卡车
    num = 0
    for j_name in network.Customer:  # 虚拟客户点j
        j_index = network.Customer.index(j_name)  # 这个虚拟客户点的索引
        expr = LinExpr(0)
        for v in vehicleLst:
            for _ in network.stsVehicleArcList:
                i = _.from_space
                j = _.to_space
                t = _.from_time
                s = _.to_time
                w = _.from_state
                w_ = _.to_state
                tpye = _.arcType
                if (tpye==4) and (j==j_name):
                    expr.addTerms(1, X[v][i][j][t][s][w][w_])  # 不满足时间窗的服务弧压根就不会构建

            for _ in network.PotentialSortiesList:
                i = _.start_space
                j = _.delivery_space  # 该架次的虚拟节点的索引
                k = _.end_space
                t = _.start_time
                s = _.delivery_time  # 该架次的虚拟节点的时间
                p = _.end_time
                if (j == j_name) and (s in network.TimeWindowMtx[j_index]):
                    expr.addTerms(1, D[v][i][j][k][t][s][p])
        num += 1
        m.addConstr(expr >= 1, f'C9_{num}')
    print("约束9已建立完成")

    # 10.无人机飞行架次决策变量与无人机路由变量逻辑约束(1)
    num = 0
    for v in vehicleLst:
        for _ in network.PotentialSortiesList:
            i = _.start_space
            j = _.delivery_space  # 此处的j来自于架次，是虚拟的节点
            j_realistic = physicalCustomerList[CustomerLst.index(j)]  # 该虚拟节点对应的实际节点索引
            k = _.end_space
            t = _.start_time
            s = _.delivery_time
            p = _.end_time
            expr = LinExpr(0)
            expr.addTerms(1, Y[v][i][i][t-sL][t][0][1])
            expr.addTerms(1, Y[v][i][j_realistic][t][s-sS][1][1])
            expr.addTerms(1, Y[v][j_realistic][j][s-sS][s][1][1])
            expr.addTerms(1, Y[v][j][j_realistic][s][s+sS][1][1])
            expr.addTerms(1, Y[v][j_realistic][k][s+sS][p][1][1])
            expr.addTerms(1, Y[v][k][k][p][p+sR][1][0])
            expr.addTerms(-6, D[v][i][j][k][t][s][p])
            num += 1
            m.addConstr(expr >= 0, f'C10_{num}_({t}_{s}_{p})')
        print("约束10已建立完成")

    # 11.无人机飞行架次决策变量与无人机路由变量逻辑约束(2)
    num = 0
    for v in vehicleLst:
        for _ in network.PotentialSortiesList:
            i = _.start_space
            j = _.delivery_space  # 此处的j来自于架次，是虚拟节点
            j_realistic = physicalCustomerList[CustomerLst.index(j)]  # 该虚拟节点对应的实际节点索引
            k = _.end_space
            t = _.start_time
            s = _.delivery_time
            p = _.end_time
            expr = LinExpr(0)
            expr.addTerms(1, Y[v][i][i][t-sL][t][0][1])
            expr.addTerms(1, Y[v][i][j_realistic][t][s-sS][1][1])
            expr.addTerms(1, Y[v][j_realistic][j][s-sS][s][1][1])
            expr.addTerms(1, Y[v][j][j_realistic][s][s+sS][1][1])
            expr.addTerms(1, Y[v][j_realistic][k][s+sS][p][1][1])
            expr.addTerms(1, Y[v][k][k][p][p+sR][1][0])
            expr.addTerms(-6, D[v][i][j][k][t][s][p])
            num += 1
            m.addConstr(expr <= 5, f'C11_{num}_({t}_{s}_{p})')
    print("约束11已建立完成")

    # 12.卡车组容量约束
    num = 0
    for v in vehicleLst:
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
                if (tpye==4) and (j==j_name):
                    expr1 += X[v][i][j][t][s][w][w_]  # 不满足时间窗的服务弧压根就不会构建

            expr2 = 0
            for _ in network.PotentialSortiesList:
                i = _.start_space
                j = _.delivery_space  # 该架次的虚拟节点的索引
                k = _.end_space
                t = _.start_time
                s = _.delivery_time  # 该架次的虚拟节点的时间
                p = _.end_time
                if (j == j_name) and (s in network.TimeWindowMtx[j_index]):
                    expr2 += D[v][i][j][k][t][s][p]
            expr += demand_j*(expr1+expr2)
        m.addConstr(expr <= vehicleCap[v], f'C12_{num}')
    print("约束12已建立完成")

    # 13.车辆无人机路由合体约束
    num = 0
    for v in vehicleLst:
        for _ in network.stsVehicleArcRemovalList:
            i = _.from_space
            j = _.to_space
            t = _.from_time
            s = _.to_time
            w = _.from_state
            w_ = _.to_state
            expr = LinExpr(0)
            expr.addTerms(1,Y[v][i][j][t][s][w][w_])
            expr.addTerms(-1, X[v][i][j][t][s][w][w_])
            num += 1
            m.addConstr(expr == 0, f'C13_{num}')
    print("约束13已建立完成")

    # 记录求解开始时间
    start_time = time.time()
    # 求解
    # m.setParam('OutPutFlag', 0)
    m.setParam('TimeLimit', TimeLimit)
    m.setParam('Threads', 1)  # 限制线程数量
    m.setParam('Heuristics', 0) # 限制较低的启发式使用频率
    m.setParam('ParallelMode', 0)  # 禁用并行计算
    m.setParam('Presolve', 1)  # 启用预处理但设置为较低级别
    m.setParam('Cuts', 0)  # 禁用Cuts
    m.setParam('MIPGap', 0.000001)  # 设置更低的MIPGap

    # m.write('SpaceTimeState_FSTSP.lp')
    m.optimize()
    if m.status == GRB.OPTIMAL:
        print("-" * 20, "求解成功", '-' * 20)
        # 输出求解总用时
        calculateTime = time.time() - start_time
        print(f"求解时间: {calculateTime} s")
        print(f"目标函数为: {m.ObjVal}")
    else:
        print("无解")
        calculateTime = -1
    # 展示求解结果
    # getSolution(network,m)
    return m,calculateTime


'''定义解类。建立求解结果的数据结构框架，并建立将Gurobi求解结果存放在数据结构中的连接'''
class Solution:
    ObjVal = 0  # 目标函数值
    X = None  # 用于存放决策变量X_ijtsww_的值
    Y = None  # 用于存放决策变量Y_ijtsww_的值
    D = None  # 用于存放决策变量D_ijktsp的值

    def __init__(self, network, model):  # 建立类属性，解类的建立需要输入数据和模型
        self.ObjVal = model.ObjVal
        self.X = [[[[[[[0 for _ in range(0, network.stateNum)] for _ in range(0, network.stateNum)] for _ in
                range(0, network.timeNum)] for _ in range(0, network.timeNum)] for _ in range(0, network.nodeNum)] for _
             in range(0, network.nodeNum)]for _ in range(0, len(network.vehicleLst))]  # x_ijtsww'_v
        self.Y = [[[[[[[0 for _ in range(0, network.stateNum)] for _ in range(0, network.stateNum)] for _ in
                range(0, network.timeNum)] for _ in range(0, network.timeNum)] for _ in range(0, network.nodeNum)] for _
             in range(0, network.nodeNum)]for _ in range(0, len(network.vehicleLst))]  # y_ijtsww'_v
        self.D = [[[[[[[0 for _ in range(0, network.timeNum)] for _ in range(0, network.timeNum)] for _ in
                range(0, network.timeNum)] for _ in range(0, network.nodeNum)] for _ in range(0, network.nodeNum)] for _
             in range(0, network.nodeNum)]for _ in range(0, len(network.vehicleLst))]  # d_ijktsp_v


def getSolution(data, model):  # 定义函数，从Gurobi输出的模型中获得目标函数值、决策变量X和Y、继而分别得到卡车和无人机路由的节点序列
    solution = Solution(data, model)
    # 在三维矩阵结构中存储自变量的取值，拆字段，检验是否是x，然后保存
    var_lst = model.getVars()
    # print('var_lst:')
    for i in var_lst:
        if round(i.x,4) != 0:
            print(i)
    for v in model.getVars():
        split_arr = re.split(r"_", v.VarName)  # 将gurobi形式的变量进行拆解，便于利用数据结构实现实现存储
        if split_arr[0] == 'X' and round(v.x,4) != 0:
            solution.X[int(split_arr[1])][int(split_arr[2])][int(split_arr[3])][int(split_arr[4])][int(split_arr[5])][int(split_arr[6])] = v.x  # X_ijtsww_
        if split_arr[0] == 'Y' and round(v.x,4) != 0:
            solution.Y[int(split_arr[1])][int(split_arr[2])][int(split_arr[3])][int(split_arr[4])][int(split_arr[5])][int(split_arr[6])] = v.x  # Y_ijtsww_
        if split_arr[0] == 'D' and round(v.x,4) != 0:
            solution.D[int(split_arr[1])][int(split_arr[2])][int(split_arr[3])][int(split_arr[4])][int(split_arr[5])][int(split_arr[6])] = v.x  # D_ijktsp
        if split_arr[0] == 'Z' and round(v.x,4) != 0:
            solution.Z[int(split_arr[1])] = v.x  # Z_v
        else:
            pass

    '''var_lst = model.getVars()
    print('var_name:')
    for i in var_lst:
        print(i.VarName)'''
    # print("车辆路由变量 solution.X:", solution.X)
    # print("无人机路由变量 solution.Y:", solution.Y)
    # print("架次决策变量 solution.D:", solution.D)

    return solution


if __name__ == "__main__":
    TimeLimit = 7200
    factor = 0
    CustomerLst = [25, 26, 27, 28, 29]  # 虚拟客户点id
    physicalCustomerList = [4, 12, 14, 18, 2]  # 实际的客户点id
    demand = [4,7,3,5,6]  # 客户需求量
    O = 0
    D = 30
    Vehicle_Cap = [55, 55, 55]  # 卡车组容量
    Truckcapacity = [50, 50, 50]  # 卡车容量
    UAVcapacity = [5, 5, 5]  # 无人机容量
    UAVendurance = [15, 15, 15]  # 无人机续航
    vehicleNum = 3  # 车辆数量
    sL = 1
    sR = 1
    sS = 1

    vehicleLst = [i for i in range(0,vehicleNum)]
    st_network = TDconstruct5.network
    st_network.readStsNode(physicalCustomerList,CustomerLst,vehicleLst, demand, Vehicle_Cap, Truckcapacity,UAVcapacity, UAVendurance,sL, sR, sS, O, D)
    st_network.readStsVehicleArc()
    st_network.readPotentialSorties(sL, sR)
    st_network.readStsDroneArc()

    model,calculate_time = modelingAndSolve(st_network)

    # 记得要新建好对应位置、对应名字的空文件夹
    instanceName = 'timeDependent_SFnetwork_5customer'
    result_Path = fr'{RESULT_DIR}/Performance of the proposed algorithm/{instanceName}/result(gurobi).txt'
    result_File = open(result_Path, 'w').close()
    result_generate = open(result_Path, mode='a')
    result_generate.write(str('Calculate_time:') + ',' + str(calculate_time))
    result_generate.write('\n')

    result_generate.write('\n')
    result_generate.write(str('OBJ_value:') + ',' + str(model.ObjVal))

    result_generate.write('\n\n')
    result_generate.write(str('Variable:'))
    result_generate.write('\n')
    for var in model.getVars():
        if round(var.X,4) != 0:
            result_generate.write(str(var.VarName) + ',' + str(var.X))
            result_generate.write('\n')
    result_generate.close()
