# -*- coding: utf-8 -*-
# Plot of the log-normal congestion index of Equation (46).
# Original file name in the authors' project: 时变拥堵指数函数绘制.py (last edited 2025/6/11)
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
import numpy as np
import matplotlib.pyplot as plt

# Define the function f(t)
def f(t):
    mu = np.log(40)
    sigma = 0.6
    coefficient = 40 / (t * sigma * np.sqrt(2 * np.pi))
    exponent = -((np.log(t) - mu) ** 2) / (2 * sigma ** 2)
    return coefficient * np.exp(exponent) + 1.0

# Generate x values from 0.1 to 100 to avoid log(0)
t_values = np.linspace(0.1, 100, 1000)
f_values = f(t_values)

# Provided scatter points
scatter_y = np.array([
    0.918800165, 1.062651351, 1.000295478, 1.062314306, 0.958010655,
    0.968806187, 1.024882464, 1.177454402, 1.123920635, 1.113735067,
    1.197153433, 1.331497772, 1.324831363, 1.441057983, 1.436865073,
    1.551329153, 1.581598105, 1.55596097, 1.714412464, 1.636516855,
    1.669973885, 1.7828882, 1.713342868, 1.829977057, 1.862867652,
    1.715114423, 1.706428293, 1.810822246, 1.702804454, 1.73197321,
    1.798913347, 1.722993825, 1.802066255, 1.673764141, 1.695002676,
    1.696878951, 1.622972466, 1.767145858, 1.656945768, 1.742003801,
    1.623237534, 1.665751536, 1.522038382, 1.599879643, 1.60514683,
    1.547602305, 1.520300124, 1.471086841, 1.440002245, 1.409880048,
    1.454748514, 1.518831044, 1.415046701, 1.363010699, 1.397634836,
    1.502927899, 1.465996018, 1.299642999, 1.348370609, 1.335878848,
    1.333966178, 1.404829738, 1.266765537, 1.218668614, 1.315033199,
    1.312452839, 1.325020519, 1.295328771, 1.165869759, 1.34423537,
    1.309817277, 1.186707009, 1.177896003, 1.200575652, 1.298237347,
    1.125372515, 1.239572644, 1.224029318, 1.157334232, 1.231279214,
    1.173056241, 1.11755745, 1.199975153, 1.188201838, 1.115330183,
    1.102453054, 1.120763514, 1.157354822, 1.074920434, 1.154554003,
    1.09904938, 1.101300608, 1.049901927, 1.045447762, 1.14643273,
    1.175751627, 1.071699431, 1.128671294, 1.075762542, 1.065668664
])



# Plot the function and the scatter points
plt.figure(figsize=(8, 5))
plt.grid(True, zorder=0,linewidth=0.5,linestyle='--',alpha=0.7)
plt.plot(t_values, f_values, label='Function $f(t)$', color='blue', zorder=1)
# Create corresponding x-values (assuming uniform spacing over [0,100])
scatter_x = np.linspace(0, 100, len(scatter_y))
plt.scatter(scatter_x, scatter_y, color='red', s=15, label='Perturbed samples', zorder=2)
plt.ylim(0.8, 2.0)  # 限制纵轴范围
plt.xlim(0, 100)  # 限制纵轴范围
plt.title('Function $f(t)$ and Superimposed Scattered Data Points')
plt.xlabel('t')
plt.ylabel('f(t)')
# plt.legend()
plt.tight_layout()
plt.show()
