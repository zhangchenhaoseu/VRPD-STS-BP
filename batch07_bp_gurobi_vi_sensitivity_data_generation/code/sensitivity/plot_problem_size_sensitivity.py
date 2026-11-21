# -*- coding: utf-8 -*-
# Plot of solution time against problem size (values typed in).
# Original file name in the authors' project: 新增问题规模敏感性分析.py (last edited 2025/6/3)
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
import os
os.environ["TMP"] = "C:/temp"
os.environ["TEMP"] = "C:/temp"

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

# 原始数据
data = np.array([
    [13, 3, 361.683803],
    [13, 4, 535.327286],
    [13, 5, 744.815955],
    [13, 6, 976.340599],
    [13, 7, 1234.308413],
    [13, 8, 1556.033648],
    [13, 9, 1850.434274],
    [14, 3, 456.879215],
    [14, 4, 657.847307],
    [14, 5, 961.356136],
    [14, 6, 1188.179772],
    [14, 7, 1520.482275],
    [14, 8, 1913.310256],
    [14, 9, 2283.220823],
    [15, 3, 576.754919],
    [15, 4, 870.567919],
    [15, 5, 1168.2],
    [15, 6, 1533.410047],
    [15, 7, 1920.361768],
    [15, 8, 2398.101706],
    [15, 9, 4113.088187],
    [16, 3, 919.928503],
    [16, 4, 1402.075636],
    [16, 5, 1932.701914],
    [16, 6, 2448.931355],
    [16, 7, 3113.536361],
    [16, 8, 3864.522389],
    [16, 9, 5005.104455],
    [17, 3, 960.474697],
    [17, 4, 1441.152765],
    [17, 5, 2135.902985],
    [17, 6, 2811.496597],
    [17, 7, 3524.580748],
    [17, 8, 4291.138865],
    [17, 9, 5305.675154],
    [18, 3, 1159.679911],
    [18, 4, 1809.006641],
    [18, 5, 2462.446753],
    [18, 6, 3219.896667],
    [18, 7, 4211.515542],
    [18, 8, 5166.701605],
    [18, 9, 6153.913285],
    [19, 3, 1300.622007],
    [19, 4, 2009.974733],
    [19, 5, 2911.393017],
    [19, 6, 3652.389403],
    [19, 7, 5042.418706],
    [19, 8, 5883.605325],
    [19, 9, 7875.363652],
])

# 拆分为坐标和数值
x, y, z = data[:, 0], data[:, 1], data[:, 2]

# 创建网格
xi = np.linspace(min(x), max(x), 100)
yi = np.linspace(min(y), max(y), 100)
xi, yi = np.meshgrid(xi, yi)

# 插值
zi = griddata((x, y), z, (xi, yi), method='cubic')

# 计算梯度
dz_dxi, dz_dyi = np.gradient(zi, xi[0, 1] - xi[0, 0], yi[1, 0] - yi[0, 0])

# 创建一个掩码，屏蔽 xi==a 或 yi==b 的箭头
mask = (xi == 19) | (yi == 9)

# 将这些位置的箭头设为0，表示不显示
dz_dxi[mask] = 0
dz_dyi[mask] = 0


# 计算每个原始数据点上的梯度值
ux = []
uy = []
for xi0, yi0 in zip(x, y):
    # 找到最近的网格点索引
    idx = (np.abs(xi[0] - xi0)).argmin()
    idy = (np.abs(yi[:, 0] - yi0)).argmin()
    ux.append(dz_dxi[idy, idx])
    uy.append(dz_dyi[idy, idx])
ux = np.array(ux)
uy = np.array(uy)

# 绘图
plt.figure(figsize=(8, 6))
contour = plt.contourf(xi, yi, zi, levels=20, cmap='Spectral_r')
plt.colorbar(contour, label='Z value')
plt.quiver(x, y, ux, uy, color='black', scale=10000, width=0.003, label='Gradient')
plt.scatter(x, y, c='white', edgecolors='black', label='Data points')



plt.xlabel('X')
plt.ylabel('Y')
plt.title('Contour Plot with Gradient Arrows and Components')
plt.tight_layout()

# 添加每个点的梯度分量标注
for xi0, yi0, uxi, uyi in zip(x, y, ux, uy):
    plt.text(xi0 + 0.1, yi0 + 0.1, f'({uxi:.1f}, {uyi:.1f})', fontsize=7, color='black')
plt.show()
