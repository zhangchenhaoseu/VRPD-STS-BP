# -*- coding: utf-8 -*-
"""Comparison with a customer-service network (Section 6.3, Table 12): launch and recovery restricted to the
depot and the customer locations. Builds an isolated copy of the project under ROOT/abs/ without touching the
main code, data, results or logs.

Instances: the 26 Sioux Falls instances with 1 to 13 customers (both settings) differ from the main instances in
one point only: the candidate rendezvous set {1, 2, 5, 8, 10, 12, 13, 14, 17, 18, 20, 22} is replaced by
{1} united with (that set intersected with the customer locations of the instance), so that the pass-through nodes
5, 8 and 17 and the nodes 20 and 22 without a customer are excluded. Time windows, demands, TTB, speed profiles
and capacities are identical (STS_node_spaceAndTime.txt and the OD files are byte-identical); the operation set is a
subset of the main one, so the optimal value cannot be lower than in the main experiment.

Before running this script, extract the data archives so that
    ROOT/abs/instanceData/<instance>/...                                   (7 network files + rendezvous_restricted.json)
    ROOT/abs/result/Performance of acceleration components/<instance>/iniOperation.txt
exist (see README, data archives instanceData_abs.zip and results_abs.zip).

What it creates:
    ROOT/abs/code/              copies of the shared modules, code/networks/, batch_common.py, a repo_paths.py with
                                DATA_DIR = ROOT/abs/instanceData and RESULT_DIR = ROOT/abs/result, and run_abs_all.py
    ROOT/abs/code/drivers/      copies of the SF drivers 1 to MAX_SF (both settings); with BLOCKS = 'parallel' only
                                the B&P-DP-VI-P block is kept (the comparison needs the optimal value and the time only)
Usage: python setup_abs.py   then   python abs/code/run_abs_all.py

Original file: setup_abs.py
"""
import io
import os
import shutil
import sys

MAX_SF = 13          # 1 to 13 customers are solved to optimality within the limit under both settings
BLOCKS = 'parallel'  # 'parallel': only B&P-DP-VI-P; 'all': the three blocks

CODE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(CODE_DIR)
DRIVER_DIR = os.path.join(CODE_DIR, 'drivers')
DST_DIR = os.path.join(ROOT_DIR, 'abs')
DST_CODE = os.path.join(DST_DIR, 'code')
DST_DATA = os.path.join(DST_DIR, 'instanceData')
DST_RESULT = os.path.join(DST_DIR, 'result', 'Performance of acceleration components')

B1 = 'print("————————————————（1）分支定价(子问题DP)'
B2 = 'print("————————————————（2）分支定价'
B3 = 'print("————————————————（3）分支定价'

REPO_PATHS = '''# -*- coding: utf-8 -*-
"""repo_paths.py of the rendezvous-restricted experiment (written by setup_abs.py): data and results live in abs/."""
import os
import sys

CODE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(CODE_DIR)                    # .../abs
DATA_DIR = os.environ.get('VRPD_DATA_DIR') or os.path.join(ROOT, 'instanceData')
RESULT_DIR = os.environ.get('VRPD_RESULT_DIR') or os.path.join(ROOT, 'result')

for _p in (os.path.join(CODE_DIR, 'networks'), CODE_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)
'''

RUNNER = '''# -*- coding: utf-8 -*-
"""Batch runner of the rendezvous-restricted experiment (written by setup_abs.py): SF drivers 1 to __MAX_SF__ of both
settings. Options as run_all.py: --list, --summary, --only=SF_td_11,SF_12. Logs and markers in abs/code/run_logs/."""
import os
import sys

from repo_paths import RESULT_DIR
import batch_common as bc

MAX_SF = __MAX_SF__
LOG_DIR = os.path.join(bc.CODE_DIR, 'run_logs')
ACC_RESULT_DIR = os.path.join(RESULT_DIR, 'Performance of acceleration components')
SUMMARY_CSV = os.path.join(RESULT_DIR, 'metrics_summary_abs.csv')


def main():
    jobs = bc.plan(MAX_SF, with_nine=False)
    if '--list' in sys.argv:
        for i, s in enumerate(jobs, 1):
            print('%2d. %s' % (i, s))
        return
    if '--summary' in sys.argv:
        bc.collect(ACC_RESULT_DIR, SUMMARY_CSV)
        return
    bc.run_jobs(jobs, bc.DRIVER_DIR, LOG_DIR, label='rendezvous restricted to depot and customers: ')
    bc.collect(ACC_RESULT_DIR, SUMMARY_CSV)


if __name__ == '__main__':
    main()
'''


def script_name(n, td):
    return 'bp_SF%s_%d.py' % ('_td' if td else '', n)


def instance_name(n, td):
    return '%sSFnetwork_%dcustomer' % ('timeDependent_' if td else '', n)


def keep_blocks(src, script):
    """Keep only the parallel block (3). The three blocks appear in different orders in the drivers, so cut by position."""
    pos = {}
    for k, b in (('1', B1), ('2', B2), ('3', B3)):
        a = src.find(b)
        if a < 0:
            raise RuntimeError('%s: start of block (%s) not found' % (script, k))
        pos[k] = src.rfind('\n', 0, a) + 1
    order = sorted(pos, key=pos.get)
    head = src[:pos[order[0]]]
    blocks = {}
    for i, k in enumerate(order):
        end = pos[order[i + 1]] if i + 1 < len(order) else len(src)
        blocks[k] = src[pos[k]:end]
    return head + blocks['3'].rstrip('\n') + '\n'


def main():
    jobs = [(script_name(n, td), instance_name(n, td)) for n in range(1, MAX_SF + 1) for td in (False, True)]
    missing = [s for s, _ in jobs if not os.path.exists(os.path.join(DRIVER_DIR, s))]
    if missing:
        print('drivers missing in code/drivers:', missing)
        return 1
    missing = [i for _, i in jobs if not os.path.exists(os.path.join(DST_DATA, i, 'STS_node_spaceAndTime.txt'))]
    if missing:
        print('instances missing in abs/instanceData (extract instanceData_abs.zip first):', missing)
        return 1
    missing = [i for _, i in jobs if not os.path.exists(os.path.join(DST_RESULT, i, 'iniOperation.txt'))]
    if missing:
        print('iniOperation.txt missing in abs/result (extract results_abs.zip first):', missing)
        return 1

    outputs = {}
    for script, inst in jobs:
        src = open(os.path.join(DRIVER_DIR, script), encoding='utf-8').read()
        if BLOCKS == 'parallel':
            try:
                src = keep_blocks(src, script)
            except RuntimeError as e:
                print(e)
                return 1
        try:
            compile(src, script, 'exec')
        except SyntaxError as e:
            print('%s: the modified copy is not valid Python: %s' % (script, e))
            return 1
        outputs[script] = src

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
        fh.write(REPO_PATHS)
    for script, src in outputs.items():
        with io.open(os.path.join(DST_CODE, 'drivers', script), 'w', encoding='utf-8', newline='\n') as fh:
            fh.write(src)
    with io.open(os.path.join(DST_CODE, 'run_abs_all.py'), 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(RUNNER.replace('__MAX_SF__', str(MAX_SF)))
    print('built %s' % DST_DIR)
    print('  code/: %d shared modules, the network stubs, %d driver copies (%s)'
          % (n_mod, len(outputs), 'B&P-DP-VI-P only' if BLOCKS == 'parallel' else 'all three blocks'))
    print('next: python %s' % os.path.join(DST_CODE, 'run_abs_all.py'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
