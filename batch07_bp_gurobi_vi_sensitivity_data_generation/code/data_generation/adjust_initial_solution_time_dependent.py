# -*- coding: utf-8 -*-
# Re-times a time-independent feasible route (arc or operation form) on the time-dependent travel times of an instance.
# Original file name in the authors' project: 可行解的时变调整.py (last edited 2025/5/15)
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
import pandas as pd

# 输入原来问题中的一个可行路径，关于X的（来自：子问题Gurobi）、关于AB的（来自：组件加速效果对比），然后输出对应的X、AB在时变中的结果

fileName = "timeDependent_SFnetwork_24customer"  # 需要更改
file_Path = fr'{DATA_DIR}/{fileName}/OD_vehicle.txt'
timeDependentdf = pd.read_csv(file_Path)
# print(timeDependentdf)

# ——————————————————————————X初始解的修改——————————————————————————
'''X_originPolicy = [f'X_{0}_{1}_{0}_{1}_{0}_{0}',f'X_{1}_{2}_{1}_{9}_{0}_{0}',f'X_{2}_{29}_{9}_{10}_{0}_{0}',f'X_{29}_{2}_{10}_{11}_{0}_{0}',f'X_{2}_{6}_{11}_{13}_{0}_{0}',f'X_{6}_{31}_{13}_{14}_{0}_{0}',f'X_{31}_{6}_{14}_{15}_{0}_{0}',f'X_{6}_{8}_{15}_{17}_{0}_{0}',f'X_{8}_{7}_{17}_{20}_{0}_{0}',f'X_{7}_{18}_{20}_{22}_{0}_{0}',f'X_{18}_{28}_{22}_{23}_{0}_{0}',f'X_{28}_{18}_{23}_{24}_{0}_{0}',f'X_{18}_{16}_{24}_{27}_{0}_{0}',f'X_{16}_{32}_{27}_{28}_{0}_{0}',f'X_{32}_{16}_{28}_{29}_{0}_{0}',f'X_{16}_{8}_{29}_{31}_{0}_{0}',f'X_{8}_{6}_{31}_{33}_{0}_{0}',f'X_{6}_{2}_{33}_{35}_{0}_{0}',f'X_{2}_{1}_{35}_{43}_{0}_{0}',f'X_{1}_{35}_{43}_{44}_{0}_{0}']



X_adjustPolicy = []
formerEndtime = 0 # 记录上一个弧/操作结束的时间
for i in range(0,len(X_originPolicy)):

    X_originPolicyLst = X_originPolicy[i].split("_")
    from_node = int(X_originPolicyLst[1])
    to_node = int(X_originPolicyLst[2])
    from_time = int(formerEndtime)
    travelTime = timeDependentdf.loc[(timeDependentdf['from_node_id'] == from_node) & (timeDependentdf['to_node_id'] == to_node) & (
                    timeDependentdf['time_stamp'] == from_time), "driving_time_cost"]
    # print(f'X_{from_node}_{to_node}_{from_time}_{0}_{0}')
    to_time = from_time + int(travelTime)
    formerEndtime = to_time
    # print(X_originPolicyLst)
    # print(f'X_{from_node}_{to_node}_{from_time}_{to_time}_{0}_{0}')
    X_adjustPolicy.append(f'X_{from_node}_{to_node}_{from_time}_{to_time}_{0}_{0}')

print("X_adjustPolicy = [")
for item in X_adjustPolicy:
    values = item.split("_")[1:]  # 去掉开头的'X'
    print(f"    f'X_{{{values[0]}}}_{{{values[1]}}}_{{{values[2]}}}_{{{values[3]}}}_{{{values[4]}}}_{{{values[5]}}}',")
print("]")'''

# ——————————————————————————ABC初始解的修改——————————————————————————
ABC_originPolicy =   [f'A_{[]}_{0}_{1}_{0}',f'C_{[44]}_{1}_{1}_{1}',f'B_{[]}_{1}_{2}_{3}',f'C_{[29]}_{2}_{2}_{11}',f'B_{[]}_{2}_{6}_{13}',f'C_{[31]}_{6}_{6}_{15}',f'B_{[]}_{6}_{5}_{17}',f'C_{[42]}_{5}_{5}_{20}',f'B_{[]}_{5}_{6}_{22}',f'B_{[]}_{6}_{8}_{25}',f'C_{[46]}_{8}_{8}_{27}',f'B_{[]}_{8}_{7}_{29}',f'C_{[47]}_{7}_{7}_{32}',f'B_{[]}_{7}_{18}_{34}',f'C_{[28]}_{18}_{18}_{36}',f'B_{[]}_{18}_{16}_{38}',f'C_{[32]}_{16}_{16}_{41}',f'B_{[]}_{16}_{17}_{43}',f'C_{[48]}_{17}_{17}_{45}',f'B_{[]}_{17}_{19}_{47}',f'C_{[36]}_{19}_{19}_{49}',f'B_{[]}_{19}_{17}_{51}',f'B_{[]}_{17}_{16}_{53}',f'B_{[]}_{16}_{8}_{55}',f'B_{[]}_{8}_{6}_{57}',f'B_{[]}_{6}_{2}_{59}',f'B_{[]}_{2}_{1}_{61}',f'A_{[]}_{1}_{49}_{69}']

ABC_adjustPolicy = []
formerEndtime = 0 # 记录上一个弧/操作结束的时间
for i in range(0,len(ABC_originPolicy)):

    ABC_originPolicyLst = ABC_originPolicy[i].split("_")
    from_node = int(ABC_originPolicyLst[2])
    to_node = int(ABC_originPolicyLst[3])
    from_time = int(formerEndtime)
    if ABC_originPolicyLst[0] == str("C"):
        travelTime = 2
    else:
        travelTime = timeDependentdf.loc[(timeDependentdf['from_node_id'] == from_node) & (timeDependentdf['to_node_id'] == to_node) & (timeDependentdf['time_stamp'] == from_time), "driving_time_cost"]
    print(f'{ABC_originPolicyLst[0]}_{ABC_originPolicyLst[1]}_{from_node}_{to_node}_{from_time}')
    to_time = from_time + int(travelTime)
    formerEndtime = to_time
    # print(ABC_originPolicyLst)
    # print(f'{ABC_originPolicyLst[0]}_{ABC_originPolicyLst[1]}_{from_node}_{to_node}_{from_time}')
    ABC_adjustPolicy.append(f'{ABC_originPolicyLst[0]}_{ABC_originPolicyLst[1]}_{from_node}_{to_node}_{from_time}')

print("ABC_adjustPolicy = [")
for item in ABC_adjustPolicy:
    if item.split("_")[0] == "A":
        values = item.split("_")[1:]  # 去掉开头的'A'
        print(f"    f'A_{{{values[0]}}}_{{{values[1]}}}_{{{values[2]}}}_{{{values[3]}}}',")
    if item.split("_")[0] == "B":
        values = item.split("_")[1:]  # 去掉开头的'B'
        print(f"    f'B_{{{values[0]}}}_{{{values[1]}}}_{{{values[2]}}}_{{{values[3]}}}',")
    if item.split("_")[0] == "C":
        values = item.split("_")[1:]  # 去掉开头的'C'
        print(f"    f'C_{{{values[0]}}}_{{{values[1]}}}_{{{values[2]}}}_{{{values[3]}}}',")
print("]")

