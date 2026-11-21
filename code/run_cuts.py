# -*- coding: utf-8 -*-
"""Re-run only the two blocks with valid inequalities (B&P-DP-VI and B&P-DP-VI-P) of every driver and
keep the B&P-DP results untouched.

This was used after the rounded capacity inequalities were changed to on-demand separation
(lazyrci.py), which does not affect the plain B&P-DP block. For each driver a temporary copy
without the B&P-DP block (_cutsonly_<i>.py in code/drivers/, deleted afterwards) is run, so
metrics(DP).json stays as it is while metrics(DP+Cut).json and metrics(DP+Cut+parallel).json are
overwritten; finally metrics_summary.csv is collected as in run_all.py.

Usage (same as run_all.py): python run_cuts.py [--list | --summary | --only=SF_td_11,SF_12]
Completion markers: code/run_logs/<driver>.cutsdone

Original file: run_cuts.py
"""
import io
import os
import sys
import time

import batch_common as bc
import run_all as RA

BLOCK1 = 'print("————————————————（1）分支定价(子问题DP)'
BLOCK_OTHER = ['print("————————————————（2）分支定价', 'print("————————————————（3）分支定价']


def strip_dp_block(src):
    a = src.find(BLOCK1)
    if a < 0:
        raise RuntimeError('start of the B&P-DP block not found')
    a0 = src.rfind('\n', 0, a) + 1
    ends = [src.find(b, a) for b in BLOCK_OTHER if src.find(b, a) >= 0]
    if ends:
        b0 = src.rfind('\n', 0, min(ends)) + 1
        return src[:a0] + src[b0:]
    return src[:a0]


def main():
    jobs = bc.plan(RA.MAX_SF)
    if '--list' in sys.argv:
        for i, s in enumerate(jobs, 1):
            print('%2d. %s' % (i, s))
        return
    if '--summary' in sys.argv:
        bc.collect(RA.ACC_RESULT_DIR, RA.SUMMARY_CSV)
        return
    if not os.path.isdir(RA.LOG_DIR):
        os.makedirs(RA.LOG_DIR)
    print('%d drivers (only the two blocks with valid inequalities), logs in %s\n' % (len(jobs), RA.LOG_DIR))
    t_all = time.time()
    for i, s in enumerate(jobs, 1):
        base = os.path.splitext(s)[0]
        done = os.path.join(RA.LOG_DIR, base + '.cutsdone')
        log = os.path.join(RA.LOG_DIR, base + '.cuts.log')
        if os.path.exists(done):
            print('[%2d/%d] skipped (done)   %s' % (i, len(jobs), s))
            continue
        src = open(os.path.join(bc.DRIVER_DIR, s), encoding='utf-8').read()
        tmp = '_cutsonly_%d.py' % i
        with io.open(os.path.join(bc.DRIVER_DIR, tmp), 'w', encoding='utf-8', newline='\n') as fh:
            fh.write(strip_dp_block(src))
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
    bc.collect(RA.ACC_RESULT_DIR, RA.SUMMARY_CSV)


if __name__ == '__main__':
    main()
