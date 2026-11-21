# -*- coding: utf-8 -*-
"""Capacity experiment of Section 6.2.3 (Table 11): build an isolated copy of the project in which the
Sioux Falls drivers run with the group capacity CAP instead of 55, without touching the main code,
results or logs.

What it creates (ROOT is the repository root):
    ROOT/cap<CAP>/code/                 copies of the shared modules, code/networks/ and batch_common.py,
                                        a repo_paths.py that keeps DATA_DIR on ROOT/instanceData (read only)
                                        and sends RESULT_DIR to ROOT/cap<CAP>/result,
                                        and run_cap_all.py (the batch runner, same options as run_all.py)
    ROOT/cap<CAP>/code/drivers/         copies of the SF drivers 1 to MAX_SF (both settings) that differ from the
                                        originals in two places only:
                                          1. Vehicle_Cap = [55, 55, 55]  ->  [CAP, CAP, CAP]
                                          2. for the instances listed in NEW_INIT (whose truck-only initial solution is
                                             overloaded at the new capacity) iniBase, iniCost and the initial columns of the
                                             three modelling functions are replaced by a feasible truck-only solution built
                                             with the pricing DP on the instance network (Appendix B rule: pure truck routes,
                                             time windows, TTB and capacity respected, every operation taken from the
                                             instance's iniOperation.txt)
    ROOT/cap<CAP>/result/Performance of acceleration components/<instance>/iniOperation.txt
                                        copied from the main result folder (the operation set does not depend on the capacity)
Every replacement is counted and the script stops without writing anything when a count is off.

CAP = 30: instances 1 to 16 (initial solution replaced for the 16-customer instances), as in Table 11.
CAP = 25: instances 1 to 14 (initial solution replaced for 13 and 14 customers). The cap25 results reported in
          Table 11 (1 to 12 customers) were produced with run_cap.py instead; this variant extends them to 13 and 14.

Usage: set CAP below and run   python setup_cap.py   then run   python cap<CAP>/code/run_cap_all.py
(an existing cap<CAP>/code is rebuilt; existing results in cap<CAP>/result are kept).

Original files: setup_cap30.py and setup_cap25.py (merged; the NEW_INIT data are copied verbatim).
"""
import io
import json
import os
import re
import shutil
import sys

CAP = 30           # 30 (truck 25 + drone 5) or 25 (truck 20 + drone 5)
MAX_SF = {30: 16, 25: 14}[CAP]

CODE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(CODE_DIR)
DRIVER_DIR = os.path.join(CODE_DIR, 'drivers')
SRC_RESULT = os.path.join(ROOT_DIR, 'result', 'Performance of acceleration components')
DST_DIR = os.path.join(ROOT_DIR, 'cap%d' % CAP)
DST_CODE = os.path.join(DST_DIR, 'code')
DST_RESULT = os.path.join(DST_DIR, 'result', 'Performance of acceleration components')

# Replacement initial solutions, {capacity: {instance: {'iniBase', 'iniCost', 'routes', 'loads', 'cap'}}},
# copied verbatim from the authors' setup_cap30.py / setup_cap25.py.
NEW_INIT_ALL = json.loads(r'''{"25":{"SFnetwork_13customer":{"iniBase":[[1,1,0,0,0,1,0,0,0,0,0,0,1,1,0,0],[0,0,1,0,0,0,0,0,1,1,1,0,0,0,1,0],[0,0,0,1,1,0,1,1,0,0,0,1,0,0,0,1]],"iniCost":[62,48,54],"routes":[[0,["A_[]_0_1_0","B_[]_1_3_1","B_[]_3_4_3","C_[25]_4_4_6","B_[]_4_3_8","B_[]_3_12_11","C_[26]_12_12_15","B_[]_12_13_17","C_[30]_13_13_26","B_[]_13_24_28","B_[]_24_21_31","B_[]_21_20_34","C_[37]_20_20_37","B_[]_20_22_39","B_[]_22_15_43","B_[]_15_14_45","B_[]_14_11_48","B_[]_11_12_52","B_[]_12_3_55","B_[]_3_1_59","A_[]_1_38_61"]],[1,["A_[]_0_1_0","B_[]_1_3_1","B_[]_3_4_3","B_[]_4_11_6","C_[33]_11_11_10","B_[]_11_10_12","C_[34]_10_10_15","B_[]_10_11_17","B_[]_11_14_20","C_[27]_14_14_24","B_[]_14_15_26","C_[35]_15_15_29","B_[]_15_14_31","B_[]_14_11_34","B_[]_11_12_38","B_[]_12_3_41","B_[]_3_1_45","A_[]_1_38_47"]],[2,["A_[]_0_1_0","B_[]_1_2_1","C_[29]_2_2_9","B_[]_2_6_11","C_[31]_6_6_13","B_[]_6_8_15","B_[]_8_16_17","B_[]_16_18_19","C_[28]_18_18_22","B_[]_18_16_24","C_[32]_16_16_27","B_[]_16_17_29","B_[]_17_19_31","C_[36]_19_19_33","B_[]_19_17_35","B_[]_17_16_37","B_[]_16_8_39","B_[]_8_6_41","B_[]_6_2_43","B_[]_2_1_45","A_[]_1_38_53"]]],"loads":[19,21,24],"cap":25},"timeDependent_SFnetwork_13customer":{"iniBase":[[0,1,0,0,0,1,0,0,1,0,0,0,0,1,0,0],[1,0,1,0,0,0,0,0,0,1,1,0,1,0,1,0],[0,0,0,1,1,0,1,1,0,0,0,1,0,0,0,1]],"iniCost":[64,84,76],"routes":[[0,["A_[]_0_1_0","B_[]_1_3_1","B_[]_3_12_3","C_[26]_12_12_7","B_[]_12_11_9","C_[33]_11_11_13","B_[]_11_12_15","B_[]_12_13_20","C_[30]_13_13_38","B_[]_13_12_40","B_[]_12_3_54","B_[]_3_1_60","A_[]_1_38_63"]],[1,["A_[]_0_1_0","B_[]_1_3_1","B_[]_3_4_3","C_[25]_4_4_6","B_[]_4_11_8","B_[]_11_10_13","C_[34]_10_10_18","B_[]_10_11_20","B_[]_11_14_26","C_[27]_14_14_34","B_[]_14_15_36","C_[35]_15_15_41","B_[]_15_22_43","B_[]_22_20_46","C_[37]_20_20_52","B_[]_20_22_54","B_[]_22_15_60","B_[]_15_14_63","B_[]_14_11_67","B_[]_11_12_72","B_[]_12_3_76","B_[]_3_1_81","A_[]_1_38_83"]],[2,["A_[]_0_1_0","B_[]_1_2_1","C_[29]_2_2_10","B_[]_2_6_12","C_[31]_6_6_15","B_[]_6_8_17","B_[]_8_16_21","B_[]_16_18_25","C_[28]_18_18_31","B_[]_18_16_33","C_[32]_16_16_39","B_[]_16_17_41","B_[]_17_19_44","C_[36]_19_19_47","B_[]_19_17_49","B_[]_17_16_52","B_[]_16_8_55","B_[]_8_6_58","B_[]_6_2_61","B_[]_2_1_64","A_[]_1_38_75"]]],"loads":[17,23,24],"cap":25},"SFnetwork_14customer":{"iniBase":[[0,1,0,0,0,1,0,0,1,0,0,0,0,1,1,0,0],[1,0,1,0,0,0,0,0,0,1,1,0,1,0,0,1,0],[0,0,0,1,1,0,1,1,0,0,0,1,0,0,0,0,1]],"iniCost":[58,62,54],"routes":[[0,["A_[]_0_1_0","B_[]_1_3_1","B_[]_3_12_3","C_[26]_12_12_7","B_[]_12_11_9","C_[33]_11_11_12","B_[]_11_12_14","B_[]_12_13_17","C_[30]_13_13_26","B_[]_13_24_28","B_[]_24_21_31","C_[38]_21_21_34","B_[]_21_24_36","B_[]_24_13_39","B_[]_13_12_42","B_[]_12_3_51","B_[]_3_1_55","A_[]_1_39_57"]],[1,["A_[]_0_1_0","B_[]_1_3_1","B_[]_3_4_3","C_[25]_4_4_6","B_[]_4_11_8","B_[]_11_10_12","C_[34]_10_10_15","B_[]_10_11_17","B_[]_11_14_20","C_[27]_14_14_24","B_[]_14_15_26","C_[35]_15_15_29","B_[]_15_22_31","B_[]_22_20_33","C_[37]_20_20_37","B_[]_20_22_39","B_[]_22_15_43","B_[]_15_14_45","B_[]_14_11_48","B_[]_11_12_52","B_[]_12_3_55","B_[]_3_1_59","A_[]_1_39_61"]],[2,["A_[]_0_1_0","B_[]_1_2_1","C_[29]_2_2_9","B_[]_2_6_11","C_[31]_6_6_13","B_[]_6_8_15","B_[]_8_16_17","B_[]_16_18_19","C_[28]_18_18_22","B_[]_18_16_24","C_[32]_16_16_27","B_[]_16_17_29","B_[]_17_19_31","C_[36]_19_19_33","B_[]_19_17_35","B_[]_17_16_37","B_[]_16_8_39","B_[]_8_6_41","B_[]_6_2_43","B_[]_2_1_45","A_[]_1_39_53"]]],"loads":[24,23,24],"cap":25},"timeDependent_SFnetwork_14customer":{"iniBase":[[0,1,0,0,0,1,0,0,1,0,0,0,0,1,1,0,0],[1,0,1,0,0,0,0,0,0,1,1,0,1,0,0,1,0],[0,0,0,1,1,0,1,1,0,0,0,1,0,0,0,0,1]],"iniCost":[80,84,76],"routes":[[0,["A_[]_0_1_0","B_[]_1_3_1","B_[]_3_12_3","C_[26]_12_12_7","B_[]_12_11_9","C_[33]_11_11_13","B_[]_11_12_15","B_[]_12_13_20","C_[30]_13_13_38","B_[]_13_24_40","B_[]_24_21_45","C_[38]_21_21_50","B_[]_21_24_52","B_[]_24_13_56","B_[]_13_12_60","B_[]_12_3_72","B_[]_3_1_77","A_[]_1_39_79"]],[1,["A_[]_0_1_0","B_[]_1_3_1","B_[]_3_4_3","C_[25]_4_4_6","B_[]_4_11_8","B_[]_11_10_13","C_[34]_10_10_18","B_[]_10_11_20","B_[]_11_14_26","C_[27]_14_14_34","B_[]_14_15_36","C_[35]_15_15_41","B_[]_15_22_43","B_[]_22_20_46","C_[37]_20_20_52","B_[]_20_22_54","B_[]_22_15_60","B_[]_15_14_63","B_[]_14_11_67","B_[]_11_12_72","B_[]_12_3_76","B_[]_3_1_81","A_[]_1_39_83"]],[2,["A_[]_0_1_0","B_[]_1_2_1","C_[29]_2_2_10","B_[]_2_6_12","C_[31]_6_6_15","B_[]_6_8_17","B_[]_8_16_21","B_[]_16_18_25","C_[28]_18_18_31","B_[]_18_16_33","C_[32]_16_16_39","B_[]_16_17_41","B_[]_17_19_44","C_[36]_19_19_47","B_[]_19_17_49","B_[]_17_16_52","B_[]_16_8_55","B_[]_8_6_58","B_[]_6_2_61","B_[]_2_1_64","A_[]_1_39_75"]]],"loads":[24,23,24],"cap":25}},"30":{"SFnetwork_16customer":{"iniBase":[[0,1,0,0,0,1,0,0,1,0,0,0,0,1,1,0,1,0,0],[1,0,1,0,0,0,0,0,0,1,1,0,1,0,0,1,0,1,0],[0,0,0,1,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1]],"iniCost":[60,64,54],"routes":[[0,["A_[]_0_1_0","B_[]_1_3_1","B_[]_3_12_3","C_[26]_12_12_7","B_[]_12_11_9","C_[33]_11_11_12","B_[]_11_12_14","B_[]_12_13_17","C_[30]_13_13_26","B_[]_13_24_28","C_[39]_24_24_31","B_[]_24_21_33","C_[38]_21_21_36","B_[]_21_24_38","B_[]_24_13_41","B_[]_13_12_44","B_[]_12_3_53","B_[]_3_1_57","A_[]_1_41_59"]],[1,["A_[]_0_1_0","B_[]_1_3_1","B_[]_3_4_3","C_[25]_4_4_6","B_[]_4_11_8","B_[]_11_10_12","C_[34]_10_10_15","B_[]_10_11_17","B_[]_11_14_20","C_[27]_14_14_24","B_[]_14_15_26","C_[35]_15_15_29","B_[]_15_22_31","C_[40]_22_22_33","B_[]_22_20_35","C_[37]_20_20_39","B_[]_20_22_41","B_[]_22_15_45","B_[]_15_14_47","B_[]_14_11_50","B_[]_11_12_54","B_[]_12_3_57","B_[]_3_1_61","A_[]_1_41_63"]],[2,["A_[]_0_1_0","B_[]_1_2_1","C_[29]_2_2_9","B_[]_2_6_11","C_[31]_6_6_13","B_[]_6_8_15","B_[]_8_16_17","B_[]_16_18_19","C_[28]_18_18_22","B_[]_18_16_24","C_[32]_16_16_27","B_[]_16_17_29","B_[]_17_19_31","C_[36]_19_19_33","B_[]_19_17_35","B_[]_17_16_37","B_[]_16_8_39","B_[]_8_6_41","B_[]_6_2_43","B_[]_2_1_45","A_[]_1_41_53"]]],"loads":[28,29,24],"cap":30},"timeDependent_SFnetwork_16customer":{"iniBase":[[0,1,0,0,0,1,0,0,1,0,0,0,0,1,1,0,1,0,0],[1,0,1,0,0,0,0,0,0,1,1,0,1,0,0,1,0,1,0],[0,0,0,1,1,0,1,1,0,0,0,1,0,0,0,0,0,0,1]],"iniCost":[83,86,76],"routes":[[0,["A_[]_0_1_0","B_[]_1_3_1","B_[]_3_12_3","C_[26]_12_12_7","B_[]_12_11_9","C_[33]_11_11_13","B_[]_11_12_15","B_[]_12_13_20","C_[30]_13_13_38","B_[]_13_24_40","C_[39]_24_24_45","B_[]_24_21_47","C_[38]_21_21_52","B_[]_21_24_54","B_[]_24_13_59","B_[]_13_12_63","B_[]_12_3_75","B_[]_3_1_80","A_[]_1_41_82"]],[1,["A_[]_0_1_0","B_[]_1_3_1","B_[]_3_4_3","C_[25]_4_4_6","B_[]_4_11_8","B_[]_11_10_13","C_[34]_10_10_18","B_[]_10_11_20","B_[]_11_14_26","C_[27]_14_14_34","B_[]_14_15_36","C_[35]_15_15_41","B_[]_15_22_43","C_[40]_22_22_46","B_[]_22_20_48","C_[37]_20_20_54","B_[]_20_22_56","B_[]_22_15_62","B_[]_15_14_65","B_[]_14_11_69","B_[]_11_12_74","B_[]_12_3_78","B_[]_3_1_83","A_[]_1_41_85"]],[2,["A_[]_0_1_0","B_[]_1_2_1","C_[29]_2_2_10","B_[]_2_6_12","C_[31]_6_6_15","B_[]_6_8_17","B_[]_8_16_21","B_[]_16_18_25","C_[28]_18_18_31","B_[]_18_16_33","C_[32]_16_16_39","B_[]_16_17_41","B_[]_17_19_44","C_[36]_19_19_47","B_[]_19_17_49","B_[]_17_16_52","B_[]_16_8_55","B_[]_8_6_58","B_[]_6_2_61","B_[]_2_1_64","A_[]_1_41_75"]]],"loads":[28,29,24],"cap":30}}}''')
NEW_INIT = NEW_INIT_ALL[str(CAP)]

REPO_PATHS = '''# -*- coding: utf-8 -*-
"""repo_paths.py of the isolated capacity experiment (written by setup_cap.py): the instance data stay in the main
repository, the results go to this folder."""
import os
import sys

CODE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(CODE_DIR)                    # .../cap__CAP__
MAIN_ROOT = os.path.dirname(ROOT)                   # the repository root
DATA_DIR = os.environ.get('VRPD_DATA_DIR') or os.path.join(MAIN_ROOT, 'instanceData')
RESULT_DIR = os.environ.get('VRPD_RESULT_DIR') or os.path.join(ROOT, 'result')

for _p in (os.path.join(CODE_DIR, 'networks'), CODE_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)
'''

RUNNER = '''# -*- coding: utf-8 -*-
"""Batch runner of the capacity __CAP__ experiment (written by setup_cap.py): the SF drivers 1 to __MAX_SF__ of both
settings. Options as run_all.py: --list, --summary, --only=SF_td_11,SF_12. Logs and .done markers in
cap__CAP__/code/run_logs/; delete a marker to re-run its driver."""
import os
import sys

from repo_paths import RESULT_DIR
import batch_common as bc

MAX_SF = __MAX_SF__
LOG_DIR = os.path.join(bc.CODE_DIR, 'run_logs')
ACC_RESULT_DIR = os.path.join(RESULT_DIR, 'Performance of acceleration components')
SUMMARY_CSV = os.path.join(RESULT_DIR, 'metrics_summary_cap__CAP__.csv')


def main():
    jobs = bc.plan(MAX_SF, with_nine=False)
    if '--list' in sys.argv:
        for i, s in enumerate(jobs, 1):
            print('%2d. %s' % (i, s))
        return
    if '--summary' in sys.argv:
        bc.collect(ACC_RESULT_DIR, SUMMARY_CSV)
        return
    bc.run_jobs(jobs, bc.DRIVER_DIR, LOG_DIR, label='group capacity __CAP__: ')
    bc.collect(ACC_RESULT_DIR, SUMMARY_CSV)


if __name__ == '__main__':
    main()
'''


def script_name(n, td):
    return 'bp_SF%s_%d.py' % ('_td' if td else '', n)


def instance_name(n, td):
    return '%sSFnetwork_%dcustomer' % ('timeDependent_' if td else '', n)


def key_to_fstring(k):
    """'E_[26, 34]_12_14_10' -> f'E_{[26, 34]}_{12}_{14}_{10}' (the way the drivers write operation keys)"""
    typ = k[0]
    inside = k[k.index('['):k.index(']') + 1]
    rest = k[k.index(']') + 2:].split('_')
    return "f'%s_{%s}_{%s}_{%s}_{%s}'" % (typ, inside, rest[0], rest[1], rest[2])


def replace_init(src, inst, script):
    ni = NEW_INIT[inst]
    # iniBase
    i = src.find('    iniBase = [[')
    j = src.find(']]', i)
    if i < 0 or j < 0:
        raise RuntimeError('%s: iniBase not found' % script)
    src = src[:i] + '    iniBase = ' + json.dumps(ni['iniBase']) + src[j + 2:]
    # iniCost
    src, k = re.subn(r'^    iniCost = \[.*?\]', '    iniCost = %s' % json.dumps(ni['iniCost']), src, flags=re.M)
    if k != 1:
        raise RuntimeError('%s: %d assignments of iniCost' % (script, k))
    # the initial columns of the three modelling functions
    for v in range(3):
        pat = "node.columnPool_SP[f'{%d}_Lambda_{0}'] = [" % v
        starts = [m.start() for m in re.finditer(re.escape(pat), src)]
        if len(starts) != 3:
            raise RuntimeError('%s: initial column of group %d found %d times, expected 3' % (script, v, len(starts)))
        keys = ni['routes'][v][1]
        new = pat + ', '.join(key_to_fstring(k) for k in keys) + ']'
        for s in reversed(starts):
            e = src.find("']", s + len(pat))
            if e < 0:
                raise RuntimeError('%s: end of the initial column of group %d not found' % (script, v))
            src = src[:s] + new + src[e + 2:]
    return src


def main():
    if not os.path.isdir(SRC_RESULT):
        print('result folder not found:', SRC_RESULT)
        return 1
    jobs = [(script_name(n, td), instance_name(n, td)) for n in range(1, MAX_SF + 1) for td in (False, True)]
    missing = [s for s, _ in jobs if not os.path.exists(os.path.join(DRIVER_DIR, s))]
    if missing:
        print('drivers missing in code/drivers:', missing)
        return 1
    missing = [i for _, i in jobs if not os.path.exists(os.path.join(SRC_RESULT, i, 'iniOperation.txt'))]
    if missing:
        print('iniOperation.txt missing in result/ for:', missing)
        return 1

    # 1. prepare every driver copy in memory first (all checks before anything is written)
    outputs = {}
    for script, inst in jobs:
        src = open(os.path.join(DRIVER_DIR, script), encoding='utf-8').read()
        pat = re.compile(r'^([ \t]*)Vehicle_Cap = \[\s*\d+\s*,\s*\d+\s*,\s*\d+\s*\](.*)$', re.M)
        if len(pat.findall(src)) != 1:
            print('%s: Vehicle_Cap is not assigned exactly once' % script)
            return 1
        src = pat.sub(r'\1Vehicle_Cap = [%d, %d, %d]  # [cap] capacity experiment, originally [55, 55, 55]' % (CAP, CAP, CAP), src)
        if inst in NEW_INIT:
            try:
                src = replace_init(src, inst, script)
            except RuntimeError as e:
                print(e)
                return 1
        try:
            compile(src, script, 'exec')
        except SyntaxError as e:
            print('%s: the modified copy is not valid Python: %s' % (script, e))
            return 1
        outputs[script] = src

    # 2. write the isolated project
    if os.path.isdir(DST_CODE):
        shutil.rmtree(DST_CODE)
    os.makedirs(os.path.join(DST_CODE, 'drivers'))
    n_mod = 0
    for f in os.listdir(CODE_DIR):
        if f.endswith('.py') and f not in ('repo_paths.py', 'run_all.py', 'run_cuts.py', 'run_cap.py', 'setup_cap.py', 'setup_abs.py',
                                           'check_iniOperation_manifest.py'):
            shutil.copy2(os.path.join(CODE_DIR, f), os.path.join(DST_CODE, f))
            n_mod += 1
    shutil.copytree(os.path.join(CODE_DIR, 'networks'), os.path.join(DST_CODE, 'networks'),
                    ignore=shutil.ignore_patterns('__pycache__'))
    with io.open(os.path.join(DST_CODE, 'repo_paths.py'), 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(REPO_PATHS.replace('__CAP__', str(CAP)))
    for script, src in outputs.items():
        with io.open(os.path.join(DST_CODE, 'drivers', script), 'w', encoding='utf-8', newline='\n') as fh:
            fh.write(src)
    with io.open(os.path.join(DST_CODE, 'run_cap_all.py'), 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(RUNNER.replace('__CAP__', str(CAP)).replace('__MAX_SF__', str(MAX_SF)))
    n_ops = 0
    for _, inst in jobs:
        d = os.path.join(DST_RESULT, inst)
        if not os.path.isdir(d):
            os.makedirs(d)
        dst = os.path.join(d, 'iniOperation.txt')
        if not os.path.exists(dst):
            shutil.copy2(os.path.join(SRC_RESULT, inst, 'iniOperation.txt'), dst)
            n_ops += 1
    replaced = [i for _, i in jobs if i in NEW_INIT]
    print('built %s' % DST_DIR)
    print('  code/: %d shared modules, the network stubs, %d driver copies (group capacity %d), %d with a new initial solution: %s'
          % (n_mod, len(outputs), CAP, len(replaced), replaced))
    print('  result/: %d iniOperation.txt copied' % n_ops)
    print('next: python %s' % os.path.join(DST_CODE, 'run_cap_all.py'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
