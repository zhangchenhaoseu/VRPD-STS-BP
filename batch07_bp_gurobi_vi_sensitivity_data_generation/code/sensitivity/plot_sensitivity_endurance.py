# -*- coding: utf-8 -*-
# Figure 21: total completion time against drone endurance (values typed in from the sensitivity runs).
# Original file name in the authors' project: 敏感性分析画图1.py
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
import matplotlib.pyplot as plt
import matplotlib as mpl

mpl.rcParams['font.family'] = 'serif'
mpl.rcParams['font.serif'] = ['Times New Roman']
mpl.rcParams['mathtext.fontset'] = 'stix'   # 数学符号也接近 Times
# -----------------------
# Data
# -----------------------
endurance = [5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25]

instance1 = [109, 109, 109, 109, 102, 102, 96, 96, 96, 96, 96]
instance2 = [93, 93, 93, 93, 87, 79, 79, 79, 79, 77, 77]
instance3 = [101, 101, 101, 101, 97, 97, 97, 97, 97, 97, 93]
instance4 = [97, 97, 97, 97, 97, 97, 97, 97, 97, 93, 93]
instance5 = [101, 101, 101, 101, 91, 91, 91, 91, 91, 91, 91]


# -----------------------
# Plot
# -----------------------
plt.figure(figsize=(7, 5))

plt.plot(endurance, instance1,
         linestyle='-', marker='o', linewidth=1.8, markersize=5,
         label='Customer set 1')

plt.plot(endurance, instance2,
         linestyle='--', marker='s', linewidth=1.8, markersize=5,
         label='Customer set 2')

plt.plot(endurance, instance3,
         linestyle='-.', marker='^', linewidth=1.8, markersize=5,
         label='Customer set 3')

plt.plot(endurance, instance4,
         linestyle=':', marker='d', linewidth=1.8, markersize=5,
         label='Customer set 4')

plt.plot(endurance, instance5,
         linestyle=(0, (5, 2)), marker='x', linewidth=1.8, markersize=6,
         label='Customer set 5')

# -----------------------
# Labels & Legend
# -----------------------
plt.xlabel('Drone Endurance', fontsize=12)
plt.ylabel('Objective Value', fontsize=12)
plt.xticks(endurance, fontsize=12)
plt.legend(
    loc='upper right',
    fontsize=18,          # 图例字体稍大
    handlelength=3.0,     # 关键：拉长虚线显示长度
    handletextpad=0.8,    # 线段与文字间距
    labelspacing=0.6,     # 每一行之间的间距
    frameon=False
)

# Light grid for readability
plt.grid(True, linestyle='--', linewidth=0.5, alpha=0.6)

# Tight layout for publication
plt.tight_layout()

plt.show()
