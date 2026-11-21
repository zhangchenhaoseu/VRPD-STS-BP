# -*- coding: utf-8 -*-
# Step 0 of the instance generation: time-stamped, FIFO-consistent travel times from the base network (OD_*_ini.txt -> OD_*.txt).
# Original file name in the authors' project: 第0步.addTimeStampToData.py (last edited 2024/10/8)
# 第0步，用来将初始的路网拓扑数据，打上时间戳，并得到满足FIFO的路段时间，为后续弧的建立做铺垫
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
import pandas as pd


def speed_function(t, rate, TTB):  # time Independence
    '''speedList = [10.88375948, 9.410424213, 9.997046092, 9.413409891, 10.43829726, 10.32198197, 9.757216415, 8.492897883,8.897425398, 8.978796035, 8.353148163, 7.510339268, 7.548130486, 6.939346033, 6.95959571, 6.44608527,6.322718753, 6.426896429, 5.832902063, 6.11053896, 5.988117594, 5.608876653, 5.836543394, 5.464549384,5.368067875, 5.830514783, 5.860193506, 5.522353186, 5.872664931, 5.773761362, 5.55891145, 5.80385133,5.549185539, 5.974557441, 5.899695702, 5.893172283, 6.161533981, 5.658842452, 6.035200545, 5.740515604,6.160527829, 6.003296282, 6.570136547, 6.250470181, 6.229959659, 6.461608366, 6.57764861, 6.797695227,6.944433616, 7.092801984, 6.874040361, 6.584010803, 7.066904569, 7.336699563, 7.154944725, 6.653679134,6.821300929, 7.694420706, 7.416358626, 7.485708763, 7.496441939, 7.118300336, 7.894120665, 8.205676165,7.604370755, 7.619321401, 7.547052936, 7.720047779, 8.577287403, 7.439173395, 7.634652693, 8.426679815,8.489713841, 8.329337667, 7.70275175, 8.885946537, 8.067296456, 8.169738953, 8.640546288, 8.12163471,8.524740464, 8.948085843, 8.333505886, 8.416078547, 8.965954795, 9.070681027, 8.922488886, 8.640392564,9.303014143, 8.661353192, 9.098772253, 9.080172957, 9.524699161, 9.565279453, 8.722709794, 8.505197673,9.330974446, 8.859975489, 9.295731738, 9.383779719]
    if 0 <= t <= len(speedList)-1:
        return int(speedList[int(t)])
    else:'''
    return 10
    # slidesNum = 10
    # if 0 <= t <= int(TTB/slidesNum):  # 时段1
         # return 10
    # elif int(TTB/slidesNum) < t <= int(2*TTB/slidesNum): # 时段2
         # return 10
    # elif int(2*TTB/slidesNum) < t <= int(3*TTB/slidesNum): # 时段3
         # return 10
    # elif int(3*TTB/slidesNum) < t <= int(4*TTB/slidesNum): # 时段4
         # return 10
    # elif int(4*TTB/slidesNum) < t <= int(5*TTB/slidesNum): # 时段5
         # return 10
    # elif int(5*TTB/slidesNum) < t <= int(6*TTB/slidesNum): # 时段6
         # return 10
    # elif int(6*TTB/slidesNum) < t <= int(7*TTB/slidesNum): # 时段7
         # return 10
    # elif int(7*TTB/slidesNum) < t <= int(8*TTB/slidesNum): # 时段8
         # return 10
    # elif int(8*TTB/slidesNum) < t <= int(9*TTB/slidesNum): # 时段9
         # return 10
    # elif int(9*TTB/slidesNum) < t <= int(10*TTB/slidesNum): # 时段10
         # return 10
    # else: # 处理定义的区间之外的情况
         # return 10


def calculate_travel_time(d_ij,t , speedFunction, rate,TTB):
    # 计算从 t 到 t + T的速度函数V（t）的积分函数
    def integral_of_speed(T):
        # intervals：用于近似积分的区间数量
        intervals = 1000
        delta_t = T / intervals
        integral_value = 0.0
        for i in range(intervals):
            t_i = t + i * delta_t
            integral_value += speedFunction(t_i,rate,TTB) * delta_t
        return integral_value

    # 计算确定让积分结果等于d_ij的T_ijt
    def find_travel_time(d_ij,rate):
        # T_ijt 的初始猜测值（可以优化以提高性能）
        T_guess = 1.0
        tolerance = 1e-1
        max_iterations = 10000
        iteration = 0

        while iteration < max_iterations:
            current_integral = integral_of_speed(T_guess)
            if abs(current_integral - d_ij) < tolerance:
                return T_guess
            # Adjust T_guess based on the current integral value
            T_guess += (d_ij - current_integral) / speedFunction(t + T_guess, rate,TTB)
            iteration += 1

        return None

    T_ijt = find_travel_time(d_ij,rate)
    return round(T_ijt) if T_ijt is not None else None


# 这是第0步，其中，卡车的所有数据都是满足FIFO的
def addTimeStampToData(FIFO_rate,TTB,name):
    # 把所给数据变为单一的最基础结构
    path_drone_ini = rf"{DATA_DIR}/{name}/OD_drone_ini.txt"
    path_vehicle_ini = rf"{DATA_DIR}/{name}/OD_vehicle_ini.txt"
    drone_df_ini = pd.read_csv(path_drone_ini)
    vehicle_df_ini = pd.read_csv(path_vehicle_ini)

    drone_df = drone_df_ini[:len(drone_df_ini)]
    vehicle_df = vehicle_df_ini[:len(vehicle_df_ini)]
    # print(drone_df)
    # print(vehicle_df)
    # 将其复制为TTB数量
    drone_df_repeated = pd.concat([drone_df] * TTB, ignore_index=True)
    vehicle_df_repeated = pd.concat([vehicle_df] * TTB, ignore_index=True)
    # 接在原先的df后面
    drone_df = pd.concat([drone_df, drone_df_repeated], ignore_index=True)
    vehicle_df = pd.concat([vehicle_df, vehicle_df_repeated], ignore_index=True)
    # print(drone_df)
    # print(vehicle_df)
    for i in range(0, len(drone_df)):
        time_stamp = i//(len(drone_df_ini))
        drone_df.loc[i, 'time_stamp'] = str(time_stamp)
    for i in range(0, len(vehicle_df)):
        time_stamp = i // (len(vehicle_df_ini))
        vehicle_df.loc[i, 'time_stamp'] = str(time_stamp)
    # print(drone_df)
    # print(vehicle_df)

    # 对于卡车重新计算driving_time_cost
    for i in range(0, len(vehicle_df)):
        d_ij = int(vehicle_df.loc[i, 'length'])
        t = int(vehicle_df.loc[i, 'time_stamp'])
        # print(FIFO.calculate_travel_time(d_ij, t, FIFO.speed_function))
        if d_ij == 10:
            vehicle_df.loc[i, 'driving_time_cost'] = str(int(d_ij/10))
        else:
            vehicle_df.loc[i, 'driving_time_cost'] = str(calculate_travel_time(d_ij, t, speed_function,FIFO_rate,TTB))
    # print(drone_df)
    # print(vehicle_df)

    drone_df.to_csv(rf"{DATA_DIR}/{name}/OD_drone.txt", sep=',', index=False)
    vehicle_df.to_csv(rf"{DATA_DIR}/{name}/OD_vehicle.txt", sep=',', index=False)


if __name__ == "__main__":
    name = "敏感性分析_SF无人机续航_第5组"
    FIFO_rate = 0.3
    TTB = 60
    addTimeStampToData(FIFO_rate,TTB,name)
    print(name,"跑完了")
