# -*- coding: utf-8 -*-
# Step 1: space-time-state nodes and customer time windows (STS_node.txt, STS_node_spaceAndTime.txt).
# Original file name in the authors' project: 第1步.generateStsNode.py (last edited 2024/10/9)
# 第1步，生成空间-时间-状态节点集合STS_node.txt，以及时间窗的设置文件STS_node_spaceAndTime.txt
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
import pandas as pd

def generateStsNode(customerList,name,timeWindow):
    readPath = rf'{DATA_DIR}/{name}/OD_vehicle.txt'
    savePath = rf'{DATA_DIR}/{name}/STS_node.txt'
    savePath1 = rf'{DATA_DIR}/{name}/STS_node_spaceAndTime.txt'
    readDf = pd.read_csv(readPath)
    # print(readDf)

    file = open(savePath, 'w').close()
    file_generate = open(savePath, mode='a')
    file_generate.write(str('space') + "," + str('time') + ","+ str('state')+","+str('customer'))  # 表头
    file_generate.write('\n')

    file1 = open(savePath1, 'w').close()
    file_generate1 = open(savePath1, mode='a')
    file_generate1.write(str('space') + "," + str('time')+ "," + str('timeWindow'))  # 表头，0就是不可访问，1是可访问
    file_generate1.write('\n')

    nodeList = list(pd.unique(readDf['from_node_id']))+list(pd.unique(readDf['to_node_id']))
    nodeList = list(set(nodeList))
    timeStampList = list(pd.unique(readDf['time_stamp']))
    stateList = [0,1]

    # print('timeStampList:', timeStampList)
    # print('nodeList:', nodeList)
    # print('stateList:', stateList)

    for i in nodeList:
        space = i
        if space in customerList:
            customer = 1
        else:
            customer = 0
        for t in timeStampList:
            time = t
            if space in customerList:
                cusIndex = customerList.index(space)
                window = timeWindow[cusIndex]
                if window[0] <= time <= window[1]:
                    file_generate1.write(str(space) + "," + str(time)+ "," + str(1))  # 记得把这里的1可以改成0
                else:
                    file_generate1.write(str(space) + "," + str(time) + "," + str(0))
            else:
                file_generate1.write(str(space) + "," + str(time) + "," + str(1))
            file_generate1.write('\n')  # 写入计算结果
            for w in stateList:
                state = w
                file_generate.write(str(space) + "," +str(time) + ","+str(state)+ "," +str(customer))
                file_generate.write('\n')  # 写入计算结果
    file_generate.close()
    file_generate1.close()
    print("时间窗都是1,表示这一时刻可以进行服务，如果要体现时间窗（则需要把1改成0）")


if __name__ == "__main__":
    name = "敏感性分析_SF无人机续航_第5组"
    customerList = [25, 26, 27, 28, 29, 30, 31, 32]
    timeWindow = [[0, 20], [0, 20], [0, 20], [20, 40], [5, 25], [0, 20], [10, 30], [10, 30]]
    generateStsNode(customerList, name, timeWindow)
