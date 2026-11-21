# -*- coding: utf-8 -*-
"""Capacity experiment of Section 6.2.3 (Table 11, group capacity 25): re-run the Sioux Falls
drivers with the group capacity Vehicle_Cap changed from 55 to CAP.

No new instance is generated: the time windows, the space-time-state network files and the operation
files (iniOperation.txt) do not depend on the group capacity, which acts only inside the pricing DP
(truckGroupCapacity) and in the right-hand sides of the rounded capacity inequalities.

For every driver a temporary copy _cap<CAP>_<i>.py is written next to the drivers and run; the copy
differs from the driver in three places only:
  1. Vehicle_Cap = [55, 55, 55]  ->  [CAP, CAP, CAP]
  2. the metrics(...).json / result(...).txt files are written to
     result/Performance of acceleration components (cap<CAP>)/<instance>/
     (iniOperation.txt is still read from the main result folder)
  3. with BLOCKS = 'serial' the parallel block (3) is removed
The copies are deleted afterwards. Logs and .done markers: code/run_logs_cap<CAP>/.
Summary: result/metrics_summary_cap<CAP>.csv (the reported cap25 results, 1 to 12 customers, were
produced this way; the cap30 results were produced with setup_cap.py).

Feasibility of the initial solution: with CAP = 30 the truck-only initial routes of instances 1 to 15
respect the capacity, with CAP = 25 those of instances 1 to 12; above these sizes one initial route is
overloaded and the script refuses to start (setup_cap.py replaces the initial solution instead).

Usage:
    python run_cap.py                    # run (finished instances are skipped)
    python run_cap.py --summary          # only collect the CSV
    python run_cap.py --list             # only print the order
    python run_cap.py --only=SF_td_11,SF_12

Original file: run_cap.py
"""
import io
import os
import re
import sys
import time

from repo_paths import RESULT_DIR
import batch_common as bc

CAP = 25            # group capacity (truck 20 + drone 5); the main instances use 55
MAX_SF = 12         # largest instance; at most 12 for CAP = 25 and 15 for CAP = 30
BLOCKS = 'all'      # 'all': the three algorithm blocks; 'serial': without the parallel block (3)
SUMMARY_ONLY = False

BASE_RESULT = os.path.join(RESULT_DIR, 'Performance of acceleration components')
CAP_RESULT = os.path.join(RESULT_DIR, 'Performance of acceleration components (cap%d)' % CAP)
LOG_DIR = os.path.join(bc.CODE_DIR, 'run_logs_cap%d' % CAP)
SUMMARY_CSV = os.path.join(RESULT_DIR, 'metrics_summary_cap%d.csv' % CAP)

CAP_LIMIT = {25: 12, 30: 15}      # largest instance whose original initial solution is feasible at this capacity

B1 = 'print("————————————————（1）分支定价(子问题DP)'
B2 = 'print("————————————————（2）分支定价'
B3 = 'print("————————————————（3）分支定价'


def plan():
    jobs = []
    for n in range(1, MAX_SF + 1):
        for td in (False, True):
            jobs.append((bc.script_name('sf', n, td), bc.instance_name('sf', n, td)))
    jobs = [j for j in jobs if os.path.exists(os.path.join(bc.DRIVER_DIR, j[0]))]
    only = [a[len('--only='):] for a in sys.argv if a.startswith('--only=')]
    if only:
        keys = [k for s in only for k in s.split(',') if k]
        jobs = [j for j in jobs if any(k in j[0] for k in keys)]
    return jobs


def strip_block3(src):
    """Remove the parallel block (3): from its print line to the print line of the next block (or EOF)."""
    a = src.find(B3)
    if a < 0:
        return src
    a0 = src.rfind('\n', 0, a) + 1
    ends = [src.find(b, a) for b in (B1, B2) if src.find(b, a) >= 0]
    if ends:
        b0 = src.rfind('\n', 0, min(ends)) + 1
        return src[:a0] + src[b0:]
    return src[:a0]


def make_variant(script, inst, idx):
    src = open(os.path.join(bc.DRIVER_DIR, script), encoding='utf-8').read()
    # 1. capacity
    pat = re.compile(r'^([ \t]*)Vehicle_Cap = \[\s*\d+\s*,\s*\d+\s*,\s*\d+\s*\](.*)$', re.M)
    m = pat.findall(src)
    if len(m) != 1:
        raise RuntimeError('%s: %d assignments of Vehicle_Cap, expected 1' % (script, len(m)))
    src = pat.sub(r'\1Vehicle_Cap = [%d, %d, %d]  # [cap] capacity experiment, originally [55, 55, 55]' % (CAP, CAP, CAP), src)
    # 2. result folder (only the metrics(...) and result(...) outputs; iniOperation.txt is still read from the main folder)
    old_dir = 'Performance of acceleration components/{instanceName}/'
    new_dir = 'Performance of acceleration components (cap%d)/{instanceName}/' % CAP
    k = 0
    for fn in ('metrics(', 'result('):
        c = src.count(old_dir + fn)
        src = src.replace(old_dir + fn, new_dir + fn)
        k += c
    if k < 4:
        raise RuntimeError('%s: only %d result paths found, unexpected format' % (script, k))
    # 3. blocks
    if BLOCKS == 'serial':
        src = strip_block3(src)
    compile(src, script, 'exec')
    tmp = '_cap%d_%d.py' % (CAP, idx)
    with io.open(os.path.join(bc.DRIVER_DIR, tmp), 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(src)
    d = os.path.join(CAP_RESULT, inst)
    if not os.path.isdir(d):
        os.makedirs(d)
    return tmp


def main():
    if '--list' in sys.argv:
        for i, (s, inst) in enumerate(plan(), 1):
            print('%2d. %s  ->  %s' % (i, s, inst))
        return
    if SUMMARY_ONLY or '--summary' in sys.argv:
        bc.collect(CAP_RESULT, SUMMARY_CSV)
        return
    if CAP in CAP_LIMIT and MAX_SF > CAP_LIMIT[CAP]:
        print('With CAP = %d the original initial solution is feasible only up to %d customers; '
              'lower MAX_SF or use setup_cap.py, which replaces the initial solution.' % (CAP, CAP_LIMIT[CAP]))
        return
    jobs = plan()
    missing = [inst for _, inst in jobs if not os.path.exists(os.path.join(BASE_RESULT, inst, 'iniOperation.txt'))]
    if missing:
        print('iniOperation.txt missing in the main result folder for:', missing)
        return
    if not os.path.isdir(LOG_DIR):
        os.makedirs(LOG_DIR)
    print('group capacity %d, %d drivers, results in %s\nlogs in %s\n' % (CAP, len(jobs), CAP_RESULT, LOG_DIR))
    t_all = time.time()
    for i, (s, inst) in enumerate(jobs, 1):
        base = os.path.splitext(s)[0]
        done = os.path.join(LOG_DIR, base + '.done')
        log = os.path.join(LOG_DIR, base + '.log')
        if os.path.exists(done):
            print('[%2d/%d] skipped (done)   %s' % (i, len(jobs), s))
            continue
        tmp = make_variant(s, inst, i)
        print('[%2d/%d] running %s' % (i, len(jobs), s), end='', flush=True)
        t0 = time.time()
        try:
            rc, nbytes = bc.run_one(os.path.join(bc.DRIVER_DIR, tmp), log, cwd=bc.DRIVER_DIR)
        finally:
            try:
                os.remove(os.path.join(bc.DRIVER_DIR, tmp))
            except OSError:
                pass
        dt = time.time() - t0
        if rc == 0:
            with io.open(done, 'w', encoding='utf-8') as fh:
                fh.write('%.1f s\n' % dt)
            print('  finished in %.1f s (%.1f MB of output)' % (dt, nbytes / 1048576.0))
        else:
            print('  [FAILED] exit code %d, see %s' % (rc, log))
    print('\nall done, total %.1f s' % (time.time() - t_all))
    bc.collect(CAP_RESULT, SUMMARY_CSV)


if __name__ == '__main__':
    main()
