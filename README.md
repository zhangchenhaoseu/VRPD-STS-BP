# VRPD-STS-BP

Reference code and data for the paper

> *A physical-network-based space-time-state modeling and branch-and-price framework for the time-dependent vehicle routing problem with drones*, Transportation Research Part E: Logistics and Transportation Review, accepted 2026. [Add the author list and the DOI here when the article is published.]

The repository contains the branch-and-price implementation (pricing by a space-time-operation dynamic program, lazily separated rounded capacity inequalities, parallel pricing), the Gurobi baselines, the instance generator, the instance files and the result files behind Tables 6 to 13 and Figures 21 to 23 of the paper, and the scripts of the sensitivity analyses.

The code is released as it was used for the paper: one driver script per instance, with the instance parameters and the initial solution written into the script. The only systematic changes made for the release are listed in [What was changed for the release](#what-was-changed-for-the-release); the differences between the code and the description in the paper that a reader should know about are listed in [Differences between the released code and the paper](#differences-between-the-released-code-and-the-paper). Comments inside the algorithm files are in Chinese, as in the original project; every file starts with an English header line that says what it does and gives its original file name (the complete mapping is in `FILE_MAP.md`).

## Repository layout

```
code/
  repo_paths.py                 repository-relative data and result folders (DATA_DIR, RESULT_DIR)
  stsnetwork.py                 reader of the space-time-state network of an instance (Section 3)
  networks/                     one two-line stub per instance: constructStsNetwork16.py, TDconstructStsNetwork16.py, ...
  constructOperation.py         construction of the feasible and efficient operations (a) to (e) (Section 5.4, Appendix C)
  constructOperation_fast.py    faster implementation of the same construction (identical output)
  DPforSPP_fast.py              pricing dynamic program of Section 5.4 and Appendix D, used by all reported runs
  DPforSPP_operation_fast.py, DPforSPP_operationValidCut_fast.py, DPforSPP_operationValidCut_parallel_fast.py
                                thin wrappers around DPforSPP_fast.py for the three driver blocks
  DPforSPP_operation.py, DPforSPP_operationImprove.py, DPforSPP_operationValidCut.py, DPforSPP_operationValidCut_parallel.py
                                the original (slower) dynamic programs; DPforSPP_operation.py is used by the sensitivity scripts
  lazyrci.py                    exact separation of the rounded capacity inequalities (24) (Section 5.3)
  metrics.py                    run statistics (iterations, pricing calls, bounds) and the wall-clock deadline
  mppool.py                     process pool of the parallel pricing (B&P-DP-VI-P)
  quiet.py                      optional redirection of the console output of a driver to a log file
  drivers/                      66 branch-and-price drivers: bp_9node_<n>.py, bp_9node_td_<n>.py, bp_SF_<n>.py, bp_SF_td_<n>.py
  baselines/gurobi_mip/         66 scripts: the arc-based MIP of Section 4 solved by Gurobi (Table 10)
  baselines/truck_only/         66 scripts: the truck-only MIP (no drones)
  baselines/bp_gurobi_vi/       38 scripts: B&P-Gurobi-VI, pricing by a Gurobi MIP, inequalities added a priori (Tables 6, 7, 10)
  sensitivity/                  the B&P-DP drivers and the plotting scripts of Section 6.4, the accessibility ranking of Table 13
  data_generation/              step0 to step4: from a travel-time table to the space-time-state network and the sortie set
  run_all.py, run_cuts.py       batch runners (all drivers; only the two blocks with inequalities)
  run_cap.py                    capacity 25 experiment of Table 11 (temporary driver copies)
  setup_cap.py, setup_abs.py    isolated copies of the project for the capacity 30 / 25 experiment (Table 11) and the
                                rendezvous-restricted experiment (Table 12)
  check_iniOperation_manifest.py   integrity check of the 48 Sioux Falls instances and of their operation files
tools/collect_data.py, tools/unpack_data.py   packaging and extraction of the data archives
result/metrics_summary.csv, result/metrics_summary_cap25.csv, cap30/result/metrics_summary_cap30.csv, abs/result/metrics_summary_abs.csv
                                the summaries from which Tables 6 to 12 were filled
data_archives/                  the instance and result archives (see below)
FILE_MAP.md                     original file name of every released file
```

## Requirements

* Python 3.8 to 3.11 with `numpy < 2`, `pandas`, and `gurobipy` (a Gurobi licence is needed for every script; the pip "restricted" licence suffices for the smallest instances). `matplotlib` and `scipy` are needed only by the plotting scripts.
* The reported runs used Python with Gurobi 10.0.1 and NumPy 1.x on Windows. The scripts no longer contain absolute paths and run on any operating system. With NumPy 2 the operation keys written by `constructOperation.py` contain `np.int64(...)` instead of plain integers and no longer match the operation files, so keep `numpy < 2` (`constructOperation_fast.py` is not affected).
* Python 3.12 and later print a `SyntaxWarning` for the `sep="\|"` argument of `pandas.read_csv` in the drivers; it is harmless.

```
pip install -r requirements.txt
```

## Data and result archives

The instance files (about 610 MB) and the result files (about 150 MB) are distributed as zip archives in `data_archives/` (built with `tools/collect_data.py` from the authors' project folder). Extract them into the repository root with

```
python tools/unpack_data.py           # or: python tools/unpack_data.py --list
```

| archive | content after extraction |
|---|---|
| `instanceData_9node.zip` | `instanceData/9nodeNetwork_<n>customer/`, `instanceData/timeDependent_9nodeNetwork_<n>customer/`, n = 1..9 |
| `instanceData_SF_time_independent.zip` | `instanceData/SFnetwork_<n>customer/`, n = 1..24 (the paper reports 1..16) |
| `instanceData_SF_time_dependent.zip` | `instanceData/timeDependent_SFnetwork_<n>customer/`, n = 1..24 |
| `instanceData_sensitivity_current.zip` | `instanceData/敏感性分析_*/` as they were last left in the project (see Section 6.4 below), the initial solutions of the five sensitivity customer sets, and the notes and time-window records of the instance folder (`instanceData/_notes/`) |
| `instanceData_abs.zip` | `abs/instanceData/<instance>/`: the 26 rendezvous-restricted instances of Table 12 |
| `results_main.zip` | `result/Performance of acceleration components/<instance>/` with `iniOperation.txt` (the operation set), `metrics(DP).json`, `metrics(DP+Cut).json`, `metrics(DP+Cut+parallel).json` and the corresponding `result(...).txt` files (solution, bounds, column pool); `result/Performance of the proposed algorithm/<instance>/` with `result(gurobi).txt` (Gurobi MIP) and `onlyVehicle_result(gurobi).txt` (truck only); `result(SP_gurobi).txt` for B&P-Gurobi-VI where it finished; `result/metrics_summary*.csv` |
| `results_cap25.zip` | `result/Performance of acceleration components (cap25)/` (Table 11, capacity 25) |
| `results_cap30.zip` | `cap30/result/` (Table 11, capacity 30) |
| `results_abs.zip` | `abs/result/` (Table 12) |
| `sensitivity_rendezvous_{depot,customer,network}.zip` | `sensitivity_rendezvous/<dimension>/instanceData/<instance>/` and `.../result/Performance of acceleration components/<instance>/` of the 90 instances of Section 6.4.3 |

Instance folder files (`instanceData/<instance>/`): `OD_vehicle_ini.txt`, `OD_drone_ini.txt` (base travel-time tables of the network), `OD_vehicle.txt`, `OD_drone.txt` (time-stamped travel times, output of step 0), `STS_node.txt`, `STS_node_spaceAndTime.txt` (space-time-state nodes and the customer time windows, step 1), `STS_arc_vehicle.txt`, `STS_arc_drone.txt` and their `_remove(1,1)` copies (space-time-state arcs, steps 2 and 3), `Potential_sorties.txt` (feasible drone sorties `start_space, delivery_space, end_space, start_time, delivery_time, end_time, cost`, step 4), and for the Sioux Falls instances `timeWindow_tw15.json` (the width-15 windows and the visit times of the initial solution they were anchored on).

`metrics(*).json` fields: `bb_nodes`, `cg_iters`, `pricing_calls`, `pricing_time_*`, `root_lp` (root bound after column generation), `n_rci_enumerated` (inequalities separated at the root), `n_rci_active_root`, `UB`, `LB`, `gap`, `proved_optimal`, `SP_time`, `RMP_time`, `total_time` (seconds). The algorithm names in the files are `DP` = B&P-DP, `DP+Cut` = B&P-DP-VI and `DP+Cut+parallel` = B&P-DP-VI-P.

## Running the code

### One instance

Every driver in `code/drivers/` is self-contained: it imports the network stub of its instance, reads the operation file `result/Performance of acceleration components/<instance>/iniOperation.txt` (or builds it with `constructOperation.FEoperationFunction` when the file is missing, which takes minutes to hours on the larger Sioux Falls instances), and runs the three algorithm blocks one after another, each with its own time limit:

```
cd code/drivers
python bp_SF_td_16.py
```

Block (1) B&P-DP, block (2) B&P-DP-VI and block (3) B&P-DP-VI-P write `metrics(DP).json`, `metrics(DP+Cut).json`, `metrics(DP+Cut+parallel).json` and the matching `result(...).txt` into the instance's result folder. The blocks appear in different orders in different drivers; the order does not matter. The instance parameters (customers, demands, capacities, endurance, the initial truck-only solution `iniBase`/`iniCost` and its operation sequence in the three `modelingDP*` functions) are at the end of each driver. To solve a new instance, generate its files with `code/data_generation/` (see below), add a stub to `code/networks/`, copy the closest driver and edit its parameter block.

`code/repo_paths.py` locates `instanceData/` and `result/` relative to the repository; the environment variables `VRPD_DATA_DIR` and `VRPD_RESULT_DIR` redirect them (this is how the sensitivity archives can be run in place, for example `VRPD_DATA_DIR=sensitivity_rendezvous/depot/instanceData VRPD_RESULT_DIR=sensitivity_rendezvous/depot/result`).

### All instances (Tables 6 to 10)

```
cd code
python run_all.py              # 9-node 1..9 and Sioux Falls 1..MAX_SF (16), both settings; logs in code/run_logs/
python run_all.py --list       # order only
python run_all.py --summary    # collect result/metrics_summary.csv from the metrics files
python run_cuts.py             # re-run only the two blocks with inequalities
```

Finished drivers leave a `.done` marker in `code/run_logs/`; delete it to re-run a driver. `check_iniOperation_manifest.py` verifies that the 48 Sioux Falls operation files are the ones used for the reported runs (row counts and MD5) and can reset the markers.

### Capacity experiment (Section 6.2.3, Table 11)

```
python run_cap.py              # capacity 25, instances 1..12 (as reported); writes result/Performance of acceleration components (cap25)/
python setup_cap.py            # CAP = 30 (1..16, as reported) or 25 (1..14): builds cap<CAP>/ with modified driver copies
python cap30/code/run_cap_all.py
```

`setup_cap.py` changes `Vehicle_Cap` in the driver copies and, for the instances whose truck-only initial solution is overloaded at the new capacity (16 customers at capacity 30; 13 and 14 at capacity 25), replaces the initial solution by the feasible truck-only solution recorded in the script (built with the pricing DP, Appendix B rule).

### Rendezvous restricted to the depot and the customer locations (Section 6.3.3, Table 12)

Extract `instanceData_abs.zip` and `results_abs.zip`, then

```
python setup_abs.py            # builds abs/code with driver copies that run block (3) only
python abs/code/run_abs_all.py
```

### Baselines

`baselines/gurobi_mip/mip_*.py` solve the arc-based formulation of Section 4 with Gurobi (`result/Performance of the proposed algorithm/<instance>/result(gurobi).txt`), `baselines/truck_only/truckonly_*.py` the same model without drones (`onlyVehicle_result(gurobi).txt`), and `baselines/bp_gurobi_vi/bpgurobi_*.py` the branch-and-price with the pricing problem solved as a MIP (`result(SP_gurobi).txt` in the acceleration-components folder). The B&P-Gurobi-VI scripts exist for the 9-node instances and Sioux Falls 1 to 10; the pricing MIP grows too large beyond the smallest instances (Tables 6 and 7).

### Sensitivity analyses (Section 6.4, Figures 21 to 23, Table 13)

The scripts in `code/sensitivity/` are the B&P-DP drivers that were used, one per customer set (`bp_sens_endurance_set<k>.py`, network stubs `sensitivityAnalysis_constructStsNetworkA..E`) and one per accessibility group (`bp_sens_rendezvous_<truck|drone>_<low|mid|high>.py`, stubs `...1..6`). They were used as templates: for every parameter level the instance was regenerated with `code/data_generation/` (endurance `f` in steps 3 and 4, speed ratio `factor` in step 3, candidate set `CandidateRendezvous` in steps 2 to 4), the instance name, `UAVendurance` and the initial columns in the script were edited, and the script was run. The instance files of the endurance and speed-ratio levels were overwritten level by level and were not kept; the instance and result files of the 90 rendezvous-distribution instances are in the `sensitivity_rendezvous_*` archives, and the aggregated values of all three analyses are the numbers typed into `plot_sensitivity_*.py` and listed in `instanceData/_notes/敏感性分析结果.xlsx`. `accessibility_truck.py` and `accessibility_drone.py` compute the rankings of Table 13 from `OD_vehicle_ini.txt` and `OD_drone_ini.txt` in the same folder.

### Generating an instance

`code/data_generation/step0_addTimeStampToData.py` to `step4_generatePotentialSorties.py` turn the base travel-time tables of a network (`OD_vehicle_ini.txt`, `OD_drone_ini.txt`) into the files listed above. The parameters (instance name, customers and their locations, demands, candidate rendezvous nodes, endurance `f`, speed ratio `factor`, service and launch/recovery times, TTB) are set in the `__main__` block of each step; the values that are there are those of the last instance generated in the project. Step 0 applies the log-normal congestion index of Equation (46) to produce the time-dependent travel times (`timeDependent_*` instances), step 4 applies the payload-dependent endurance test to every sortie. `adjust_initial_solution_time_dependent.py` re-times a time-independent route on the time-dependent travel times (used to obtain the initial solutions of the time-dependent instances).

## Where the paper's algorithm lives in the code

| paper | code |
|---|---|
| space-time-state network, Sections 3.3, 3.4 and Appendix A | `stsnetwork.ReadData` (reads the files of steps 1 to 4) |
| effective operations (a) to (e), Section 5.4, Figure 11, Appendix C | `constructOperation.FEoperationFunction` (and `constructOperation_fast`); the sorties themselves are generated in step 4, Section 4.1 |
| restricted master problem (19) to (23), Section 5.3 | `modelingDP*` and `columnGenerationDP*` in each driver (Gurobi LP, duals read after each solve) |
| rounded capacity inequalities (24), separation | `lazyrci.separate` (exact separation over the candidate subsets, those with demand above the largest route capacity; at most 100 most violated inequalities per round, tolerance 1e-6), called in `columnGenerationDP_CUT*` when pricing finds no column; a node is closed only when neither a column nor an inequality is found |
| pricing dynamic program, recursion (39) and (40), Appendix D | `DPforSPP_fast.shortest_path_operations*` (one label per state, chronological extension, cached multiplier increments; `USE_DOMINANCE` switches the state dominance rule of Section 5.4 on, default off, and the drivers use the default) |
| branching, Section 5.5 | `branchAndPriceDP*` in each driver (most fractional operation flow, deterministic tie order `_branch_order`; left child removes the operation and fixes the column to 0, right child adds the big-M term `adjustCoff` and the dual to the pricing objective) |
| parallel pricing, Section 5.1 and variant (5) of Section 6.1 | `mppool.get_pool` and `DPforSPP_operationValidCut_parallel_fast` (min(number of truck groups, CPU cores) worker processes, the network sent to the workers once; the pricing time of a round is the maximum over the groups) |
| time limits, Section 6.1 | `metrics.DEADLINE`, 7200 s for the 9-node and 10800 s for the Sioux Falls drivers, checked between column-generation rounds and between nodes |
| B&P-Gurobi-VI | `baselines/bp_gurobi_vi/*.py`, pricing model built in `modeling` |

## What was changed for the release

Every change below was applied mechanically to all files of a kind and verified by comparing the abstract syntax trees of the released and the original files; the algorithmic code is otherwise untouched.

1. The absolute Windows paths of the authors' machine were replaced by `DATA_DIR` and `RESULT_DIR` from `code/repo_paths.py`, and three bootstrap lines (`import os, sys, math`, `sys.path.insert(...)`, `from repo_paths import ...`) were added to every script. The personal header lines (motto, e-mail, affiliation, date) were removed; the date is kept in the English header line.
2. The 67 per-instance copies of the network reader (`constructStsNetwork*.py`, identical except for the instance name) were replaced by `code/stsnetwork.py` plus two-line stubs in `code/networks/`, so that the drivers keep their original `import` lines. The subset enumeration that the reader performed before the inequalities were separated lazily is kept as `ReadData.enumerate_subsets()`, which the B&P-Gurobi-VI scripts call because they add the inequalities a priori.
3. The eleven `sensitivityAnalysis_constructOperation*.py` copies, identical to `constructOperation.py` outside their self-test, were removed and the sensitivity scripts import `constructOperation` instead.
4. In `constructOperation.py` and `constructOperation_fast.py` the import of a network module (used only by the self-test) was moved into the `__main__` block, the candidate rendezvous list became the module constant `CANDIDATE_RENDEZVOUS` with a warning when the sortie file uses other nodes, and the optional switch `CHARGE_PAYLOAD_WHILE_HOVERING` (default `False`, the behaviour of all reported runs) was added; see the next section. The same import move was made in the four original `DPforSPP_operation*` modules.
5. In the B&P-Gurobi-VI scripts the switch `UPPER_BOUND_ON_P` (default `True`) adds the upper bound of constraint (33) of the published paper; the reported runs used the lower bound only (the form of the first submission).
6. The batch runners and the experiment set-up scripts were rewritten for the new file names (`run_all.py`, `run_cuts.py`, `run_cap.py`, `setup_cap.py`, `setup_abs.py`, `check_iniOperation_manifest.py`, `batch_common.py`); their logic is that of the original scripts, whose initial-solution data and manifest are copied verbatim.
7. The scripts were renamed (`FILE_MAP.md`); development tools (patch scripts, benchmarks, earlier prototypes, backups) are not included.

## Differences between the released code and the paper

These are the points where the code, the data or the reported numbers differ from what a reader of the paper may expect. None of them was changed after the review, so that the released material is the one behind the reported results.

1. **Payload-dependent endurance and waiting for the truck.** Step 4 of the instance generation applies the payload term, with F(d) = d, to the span of each sortie, including the drone's waiting on the flight arcs: `flyIn + flyOut <= f - S_L - S_R - 2 s_S - d`. When `constructOperation` turns a sortie into a D- or E-type operation, the drone may in addition hover at the recovery node until the truck arrives, and this extra waiting is bounded by the payload-free endurance only (`Truck_SPvalue <= UAVendurance - 2`). Rule (3) of the paper bounds the whole launch-service-recovery cycle by f - F(d). In the optimal solutions reported for the 50 main instances, 72 of the 190 sorties use this additional waiting (32 instances are affected). The Gurobi MIP baseline does not allow it, since its sorties are the pre-generated ones. Setting `CHARGE_PAYLOAD_WHILE_HOVERING = True` in `constructOperation.py` (and in `constructOperation_fast.py`) applies the paper's rule to the whole cycle; the operation files then have to be rebuilt (delete `iniOperation.txt` or run `FEoperationFunction`), and the optimal values can only increase. On `9nodeNetwork_1customer` the switch removes 115 of the 2664 operations.
2. **Candidate rendezvous set hard-coded.** `CANDIDATE_RENDEZVOUS` in `constructOperation.py` is the twelve-node set of the Sioux Falls instances of Sections 6.2 and 6.3; the 9-node operation files were built with `[1, ..., 9]`. The operation files of the rendezvous-distribution instances of Section 6.4.3 (archives `sensitivity_rendezvous_*`) were built with the twelve-node list unchanged, so in those runs only the candidate nodes of each six-node group that also belong to the twelve-node set were available for launch and recovery (for example nodes 8, 10, 13 and 14 of the group {7, 8, 10, 13, 14, 16}); the sortie files of these instances contain the sorties at all six nodes. The released module prints a warning in this situation.
3. **Time limits and `proved_optimal`.** The drivers use wall-clock deadlines (`metrics.DEADLINE`) of 7200 s (9-node) and 10800 s (Sioux Falls) checked between column-generation rounds and between nodes, so a run can end, and prove optimality, after the deadline by the length of the last round; Table 7 counts such runs as exceeding the limit (for example B&P-DP-VI on `timeDependent_SFnetwork_16customer`, whose metrics file reports a root-node proof completed at 10906 s). The Gurobi MIP baselines use `TimeLimit = 7200` for both networks, B&P-Gurobi-VI uses 3600 s on the time-independent Sioux Falls instances 5 to 10 and 7200 s elsewhere, and the truck-only scripts of the time-dependent Sioux Falls instances 20 to 24 (not reported) use 72000 s or 720000 s; `truckonly_SF_5.py` ran with the default Gurobi parameters (the parameter lines are commented out), all other baseline scripts with `Threads=1, Heuristics=0, Presolve=1, Cuts=0, MIPGap=1e-6`.
4. **Two table entries come from a second machine.** The entries of B&P-DP on `timeDependent_9nodeNetwork_9customer` (2392.43 s pricing, 1003 nodes, Tables 6 and 8) and of B&P-DP-VI-P on `timeDependent_SFnetwork_15customer` (9577.0 s total, 9327.48 s pricing, Table 7) were obtained on a second computer; the metrics files of these two runs are not in `results_main.zip`, which holds the runs made on the main machine that stopped at the time limit with gaps of 5.8% and 8.4%. In Table 7 the B&P-DP-VI-P entries of the time-independent 14- and 15-customer instances (8759.60 s and 9970.97 s) are the pricing times of runs that stopped at the time limit with gaps of 8.0% and 2.9%.
5. **Instance parameters.** The planning horizon is not 100 for every instance as stated in Section 6.1: each instance uses TTB = latest return time of its initial truck-only solution + 1 (for example 87 for `timeDependent_SFnetwork_16customer` and 40 for `9nodeNetwork_1customer`), which is the value in its `STS_node.txt`. The initial upper bound in the drivers is +inf (`globalUB = np.inf`) rather than 100; the initial truck-only solution gives the first finite bound in either case. F(d) is the identity, F(d) = d, in step 4.
6. **B&P-Gurobi-VI formulation.** The released scripts add the upper bound of constraint (33) (`UPPER_BOUND_ON_P = True`); set it to `False` to reproduce the reported runs, which used the lower bound only. On the instances that B&P-Gurobi-VI solved no inequality (24) is active, so the optimal values are unaffected.
7. **Operation files.** The 48 Sioux Falls operation files were built with `constructOperation_fast.py` (September 2026) for the width-15 time windows; the 9-node files with `constructOperation.py`. The two modules produce byte-identical files; `check_iniOperation_manifest.py` holds the row counts and MD5 sums of the 48 files.
8. **Sensitivity instances.** The endurance and speed-ratio instances were regenerated in place for each level and only the aggregated results survive (point 6.4 above); the folders `instanceData/敏感性分析_*` in `instanceData_sensitivity_current.zip` are the ones last left in the project and do not all correspond to the group named in the folder.

## Licence and citation

[Choose a licence for the code (for example MIT) and cite the paper when using the code or the instances.]
