# -*- coding: utf-8 -*-
# Figure 23: total completion time against the accessibility of the candidate rendezvous nodes (values typed in from the sensitivity runs).
# Original file name in the authors' project: 敏感性分析画图2.py
import os, sys, math
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))  # shared modules in code/
from repo_paths import DATA_DIR, RESULT_DIR  # noqa: E402  (also puts code/ and code/networks/ on sys.path)
import matplotlib.pyplot as plt
import matplotlib as mpl
import matplotlib.patches as mpatches
import numpy as np

# Set font style to match Times New Roman
mpl.rcParams['font.family'] = 'serif'
mpl.rcParams['font.serif'] = ['Times New Roman']
mpl.rcParams['mathtext.fontset'] = 'stix'

# Check matplotlib version for boxplot parameter compatibility
import matplotlib

mpl_version = tuple(map(int, matplotlib.__version__.split('.')[:2]))
use_tick_labels = mpl_version >= (3, 9)  # matplotlib 3.9+ uses 'tick_labels'

print(f"Matplotlib version: {matplotlib.__version__}")
print(f"Using {'tick_labels' if use_tick_labels else 'labels'} parameter for boxplot")

# Data organization
scenarios = [
    {
        "name": "Transportation nodes",
        "truck": {"low": [109, 87, 96, 97, 88], "medium": [102, 79, 97, 97, 91], "high": [105, 77, 97, 97, 101]},
        "drone": {"low": [106, 87, 96, 97, 88], "medium": [97, 86, 100, 97, 93], "high": [105, 77, 97, 97, 101]}
    },
    {
        "name": "Depot locations",
        "truck": {"low": [104, 93, 101, 97, 101], "medium": [105, 77, 95, 93, 93], "high": [109, 78, 88, 86, 85]},
        "drone": {"low": [102, 93, 100, 97, 101], "medium": [105, 77, 97, 93, 93], "high": [109, 78, 92, 92, 89]}
    },
    {
        "name": "Customer locations",
        "truck": {"low": [109, 93, 96, 90, 101], "medium": [100, 79, 97, 93, 97], "high": [102, 75, 93, 97, 97]},
        "drone": {"low": [109, 87, 100, 90, 101], "medium": [101, 79, 100, 89, 88], "high": [102, 75, 93, 97, 97]}
    }
]


# Calculate means for each accessibility level
def calc_means(scenario):
    truck_means = [np.mean(scenario["truck"]["low"]),
                   np.mean(scenario["truck"]["medium"]),
                   np.mean(scenario["truck"]["high"])]
    drone_means = [np.mean(scenario["drone"]["low"]),
                   np.mean(scenario["drone"]["medium"]),
                   np.mean(scenario["drone"]["high"])]
    return truck_means, drone_means


# =============================================================================
# Figure 1: Comparative analysis across scenarios
# =============================================================================
'''print("\nGenerating Figure 1...")
fig1, axes = plt.subplots(1, 3, figsize=(15, 5))
fig1.suptitle('Sensitivity Analysis: Impact of Rendezvous Node Accessibility on Objective Value',
              fontsize=16, fontweight='bold', y=1.02)

colors_truck = ['#E74C3C', '#E67E22', '#27AE60']
colors_drone = ['#C0392B', '#D35400', '#229954']

x_positions = np.arange(3)
width = 0.35

for idx, scenario in enumerate(scenarios):
    ax = axes[idx]
    truck_means, drone_means = calc_means(scenario)

    bars1 = ax.bar(x_positions - width / 2, truck_means, width, label='Truck',
                   color=colors_truck, alpha=0.8, edgecolor='black', linewidth=0.8)
    bars2 = ax.bar(x_positions + width / 2, drone_means, width, label='Drone',
                   color=colors_drone, alpha=0.8, edgecolor='black', linewidth=0.8)

    # Add value labels on bars
    for bars in [bars1, bars2]:
        for bar in bars:
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width() / 2., height,
                    f'{height:.1f}',
                    ha='center', va='bottom', fontsize=10, fontweight='bold')

    ax.set_xlabel('Accessibility Level', fontsize=12, fontweight='bold')
    ax.set_ylabel('Mean Objective Value', fontsize=12, fontweight='bold')
    ax.set_title(f'{scenario["name"]}', fontsize=12, fontweight='bold')
    ax.set_xticks(x_positions)
    ax.set_xticklabels(['Low', 'Medium', 'High'], fontsize=12)
    ax.tick_params(axis='y', labelsize=12)
    ax.legend(loc='upper right', fontsize=11, frameon=False)
    ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.6)
    ax.set_ylim([70, 110])

plt.tight_layout()
plt.savefig('figure1_comparative_analysis.png', dpi=300, bbox_inches='tight')
print("✓ Figure 1 created successfully!")
plt.close()

# =============================================================================
# Figure 2: Instance-level analysis for Depot scenario
# =============================================================================
print("Generating Figure 2...")
fig2, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
fig2.suptitle('Detailed Analysis: Depot Accessibility Impact by Instance',
              fontsize=16, fontweight='bold', y=1.00)

depot_scenario = scenarios[1]
instances = ['Inst 1', 'Inst 2', 'Inst 3', 'Inst 4', 'Inst 5']
x = np.arange(len(instances))
width = 0.25

# Truck subplot
truck_low = depot_scenario["truck"]["low"]
truck_med = depot_scenario["truck"]["medium"]
truck_high = depot_scenario["truck"]["high"]

bars1 = ax1.bar(x - width, truck_low, width, label='Low Accessibility',
                color='#E74C3C', alpha=0.8, edgecolor='black', linewidth=0.8)
bars2 = ax1.bar(x, truck_med, width, label='Medium Accessibility',
                color='#E67E22', alpha=0.8, edgecolor='black', linewidth=0.8)
bars3 = ax1.bar(x + width, truck_high, width, label='High Accessibility',
                color='#27AE60', alpha=0.8, edgecolor='black', linewidth=0.8)

ax1.set_xlabel('Customer Instance', fontsize=12, fontweight='bold')
ax1.set_ylabel('Objective Value', fontsize=12, fontweight='bold')
ax1.set_title('Truck Operations', fontsize=12, fontweight='bold')
ax1.set_xticks(x)
ax1.set_xticklabels(instances, fontsize=12)
ax1.tick_params(axis='y', labelsize=12)
ax1.legend(loc='upper right', fontsize=11, frameon=False)
ax1.grid(True, linestyle='--', linewidth=0.5, alpha=0.6)
ax1.set_ylim([70, 115])

# Drone subplot
drone_low = depot_scenario["drone"]["low"]
drone_med = depot_scenario["drone"]["medium"]
drone_high = depot_scenario["drone"]["high"]

bars1 = ax2.bar(x - width, drone_low, width, label='Low Accessibility',
                color='#C0392B', alpha=0.8, edgecolor='black', linewidth=0.8)
bars2 = ax2.bar(x, drone_med, width, label='Medium Accessibility',
                color='#D35400', alpha=0.8, edgecolor='black', linewidth=0.8)
bars3 = ax2.bar(x + width, drone_high, width, label='High Accessibility',
                color='#229954', alpha=0.8, edgecolor='black', linewidth=0.8)

ax2.set_xlabel('Customer Instance', fontsize=12, fontweight='bold')
ax2.set_ylabel('Objective Value', fontsize=12, fontweight='bold')
ax2.set_title('Drone Operations', fontsize=12, fontweight='bold')
ax2.set_xticks(x)
ax2.set_xticklabels(instances, fontsize=12)
ax2.tick_params(axis='y', labelsize=12)
ax2.legend(loc='upper right', fontsize=11, frameon=False)
ax2.grid(True, linestyle='--', linewidth=0.5, alpha=0.6)
ax2.set_ylim([70, 115])

plt.tight_layout()
plt.savefig('figure2_depot_instance_analysis.png', dpi=300, bbox_inches='tight')
print("✓ Figure 2 created successfully!")
plt.close()
'''
# =============================================================================
# Figure 3: Trend lines showing accessibility impact
# =============================================================================
print("Generating Figure 3...")
fig3, axes = plt.subplots(2, 3, figsize=(21, 10))
fig3.suptitle('Accessibility Impact Trends Across All Scenarios and Instances',
              fontsize=16, fontweight='bold', y=0.995)

accessibility_levels = ['Low', 'Medium', 'High']
x_numeric = [1, 2, 3]

# Line styles and markers
line_styles = ['-', '--', '-.', ':', (0, (5, 2))]
markers = ['o', 's', '^', 'd', 'x']
marker_sizes = [5, 5, 5, 5, 7]

for row_idx, mode in enumerate(['truck', 'drone']):
    for col_idx, scenario in enumerate(scenarios):
        ax = axes[row_idx, col_idx]

        # Plot each instance as a line
        for inst_idx in range(5):
            values = [scenario[mode]["low"][inst_idx],
                      scenario[mode]["medium"][inst_idx],
                      scenario[mode]["high"][inst_idx]]
            ax.plot(x_numeric, values,
                    linestyle=line_styles[inst_idx],
                    marker=markers[inst_idx],
                    linewidth=1.8,
                    markersize=marker_sizes[inst_idx],
                    label=f'Customer set {inst_idx + 1}',
                    alpha=0.85)

        # Add mean trend line
        low_mean = np.mean(scenario[mode]["low"])
        med_mean = np.mean(scenario[mode]["medium"])
        high_mean = np.mean(scenario[mode]["high"])
        ax.plot(x_numeric, [low_mean, med_mean, high_mean],
                marker='*',
                linewidth=2.5,
                markersize=10,
                color='black',
                label='Mean',
                linestyle='-',
                alpha=0.95)

        title_mode = mode.capitalize()
        ax.set_title(f'{title_mode} - {scenario["name"]}', fontsize=15, fontweight='bold')
        ax.set_xlabel('Accessibility level', fontsize=15)
        ax.set_ylabel('Total completion time', fontsize=15)
        ax.set_xticks(x_numeric)
        ax.set_xticklabels(accessibility_levels, fontsize=15)
        ax.tick_params(axis='y', labelsize=15)
        ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.6)
        ax.set_ylim([70, 115])

        if col_idx == 2:
            ax.legend(bbox_to_anchor=(1.05, 1),
                      loc='upper left',
                      fontsize=15,
                      handlelength=3.0,
                      handletextpad=0.8,
                      labelspacing=0.6,
                      frameon=False)

plt.tight_layout()
plt.savefig('figure3_trend_analysis.png', dpi=300, bbox_inches='tight')
print("✓ Figure 3 created successfully!")
plt.close()

# =============================================================================
# Figure 4: Box plots showing variability
# =============================================================================
'''print("Generating Figure 4...")
fig4, axes = plt.subplots(2, 3, figsize=(21, 10))
fig4.suptitle('Variability Analysis: Distribution of Objective Values by Accessibility Level',
              fontsize=16, fontweight='bold', y=0.995)

for row_idx, mode in enumerate(['truck', 'drone']):
    for col_idx, scenario in enumerate(scenarios):
        ax = axes[row_idx, col_idx]

        data_to_plot = [scenario[mode]["low"],
                        scenario[mode]["medium"],
                        scenario[mode]["high"]]

        # Use appropriate parameter based on matplotlib version
        boxplot_params = {
            'patch_artist': True,
            'widths': 0.6,
            'showmeans': True,
            'meanline': True,
            'boxprops': dict(facecolor='lightblue', alpha=0.7, linewidth=1.5),
            'medianprops': dict(color='red', linewidth=2),
            'meanprops': dict(color='green', linewidth=2, linestyle='--'),
            'whiskerprops': dict(linewidth=1.5),
            'capprops': dict(linewidth=1.5)
        }

        if use_tick_labels:
            boxplot_params['tick_labels'] = accessibility_levels
        else:
            boxplot_params['labels'] = accessibility_levels

        bp = ax.boxplot(data_to_plot, **boxplot_params)

        title_mode = mode.capitalize()
        ax.set_title(f'{title_mode} - {scenario["name"]}', fontsize=12, fontweight='bold')
        ax.set_xlabel('Accessibility Level', fontsize=12)
        ax.set_ylabel('Objective Value', fontsize=12)
        ax.tick_params(axis='both', labelsize=12)
        ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.6, axis='y')
        ax.set_ylim([70, 115])

# Add legend for box plot elements
red_patch = mpatches.Patch(color='red', label='Median')
green_line = mpatches.Patch(color='green', label='Mean')
fig4.legend(handles=[red_patch, green_line],
            loc='lower center',
            fontsize=11,
            ncol=2,
            frameon=False,
            bbox_to_anchor=(0.5, -0.02))

plt.tight_layout()
plt.savefig('figure4_variability_analysis.png', dpi=300, bbox_inches='tight')
print("✓ Figure 4 created successfully!")
plt.close()

print("\n" + "=" * 60)
print("All figures created successfully with Times New Roman style!")
print("=" * 60)'''
