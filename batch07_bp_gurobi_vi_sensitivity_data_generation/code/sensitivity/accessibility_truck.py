# -*- coding: utf-8 -*-
# Accessibility ranking of the Sioux Falls nodes for the truck (Equation (47), Table 13); reads OD_vehicle_ini.txt from the working directory.
# Original file name in the authors' project: 000.输出SF网络基于卡车的可达性结果.py (last edited 2025/11/10)
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
import pandas as pd
import heapq

def load_graph(file_path):
    """读取txt文件并构建邻接表"""
    df = pd.read_csv(file_path)
    graph = {}
    for _, row in df.iterrows():
        u, v, length = int(row['from_node_id']), int(row['to_node_id']), float(row['length'])
        if u not in graph:
            graph[u] = []
        graph[u].append((v, length))
    return graph

def dijkstra(graph, start, end):
    """计算从 start 到 end 的最短路径长度"""
    # 最小堆（优先队列）
    pq = [(0, start)]
    # 记录最短距离
    dist = {node: float('inf') for node in graph}
    dist[start] = 0

    while pq:
        current_dist, u = heapq.heappop(pq)
        if u == end:
            return current_dist  # 提前终止：已找到最短路
        if current_dist > dist[u]:
            continue
        for v, length in graph.get(u, []):
            new_dist = current_dist + length
            if new_dist < dist.get(v, float('inf')):
                dist[v] = new_dist
                heapq.heappush(pq, (new_dist, v))
    return float('inf')  # 若不可达

# ====== 示例用法 ======
file_path = 'OD_vehicle_ini.txt'   # 改成你的txt文件路径
graph = load_graph(file_path)

i, j = 0, 19   # 例如从节点0到节点19
for i in range(1,25):
    for j in range(1,25):
        shortest_length = dijkstra(graph, i, j)
        print(f"{shortest_length}")
