# -*- coding: utf-8 -*-
# Plot of the node accessibility of the Sioux Falls network.
# Original file name in the authors' project: 000.SF网络可达性出图.py (last edited 2025/11/10)
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
from scipy.interpolate import Rbf
import os
os.environ["TMPDIR"] = "C:/Temp"
os.environ["TEMP"] = "C:/Temp"
os.environ["TMP"] = "C:/Temp"
# 原始数据
truckNodes = {
    'x': [73.6,395,73.6,170,278,395,511.6,395,278,278,170,73.6,73.6,170,277.9,395,395,511.6,395,395,277.9,277.9,170,170],
    'y': [-34.3,-34.3,-118.3,-118.3,-118.3,-118.3,-187.6,-187.6,-187.6,-259.3,-259.3,-259.3,-580.8,-416.7,-416.7,-259.3,-329.9,-259.3,-416.7,-580.8,-580.8,-487.5,-487.5,-580.8],
    'accessibility': [12.5,10.75,11.04166667,9.083333333,8.333333333,9.208333333,10.375,8.208333333,7.333333333,6.833333333,7.583333333,9.541666667,13.375,8.5,7.75,7.708333333,7.75,9.875,8.25,10.41666667,10.66666667,8.75,9.5,11.41666667]
}

droneNodes = {
    'x': [73.6, 395, 73.6, 170, 278, 395, 511.6, 395, 278, 278, 170, 73.6, 73.6, 170, 277.9, 395, 395, 511.6, 395, 395,
          277.9, 277.9, 170, 170],
    'y': [-34.3, -34.3, -118.3, -118.3, -118.3, -118.3, -187.6, -187.6, -187.6, -259.3, -259.3, -259.3, -580.8, -416.7,
          -416.7, -259.3, -329.9, -259.3, -416.7, -580.8, -580.8, -487.5, -487.5, -580.8],
    'accessibility': [4.5,4.041666667,3.791666667,3.375,3.166666667,3.291666667,3.458333333,2.916666667,2.666666667,2.416666667,2.666666667,3.25,4.333333333,2.875,2.666666667,2.625,2.75,3.25,2.916666667,3.958333333,3.708333333,3.208333333,3.416666667,3.875
]
}

nodes = truckNodes

x = np.array(nodes['x'])
y = np.array(nodes['y'])
z = np.array(nodes['accessibility'])

# 创建网格
grid_x, grid_y = np.meshgrid(
    np.linspace(min(x)-2, max(x)+2, 500),
    np.linspace(min(y)-2, max(y)+2, 500)
)

# RBF 插值
rbf = Rbf(x, y, z, function='multiquadric')  # 可改为 'linear', 'gaussian', 等
grid_z = rbf(grid_x, grid_y)

# 绘图
plt.figure(figsize=(10, 8))
contour = plt.contourf(grid_x, grid_y, grid_z, levels=100, cmap='turbo')  # 色图
plt.scatter(x, y, c=z, cmap='turbo', edgecolor='black', s=50)  # 原始点
plt.colorbar(contour, label='Accessibility')
plt.title('Accessibility Interpolation (RBF)')
plt.xlabel('x')
plt.ylabel('y')
plt.axis('equal')
plt.tight_layout()
plt.show()
