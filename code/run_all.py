# -*- coding: utf-8 -*-
"""Run all B&P drivers (code/drivers/bp_*.py) one after another and collect the metrics into
result/metrics_summary.csv (the source of Tables 6 to 10).

* one log per driver in code/run_logs/, written while the run is in progress;
* a .done marker per finished driver, so an interrupted batch can be restarted and skips what is done
  (delete the marker to re-run a driver);
* after the batch (or with --summary alone) the metrics(*).json files are gathered into one CSV.

Usage:
    python run_all.py                       # run everything that is not done yet
    python run_all.py --summary             # only collect the CSV
    python run_all.py --list                # only print the order
    python run_all.py --only=SF_td_13,SF_14 # only drivers whose file name contains these sub-strings

Original file: run_all.py (the driver file names and the result root changed in the release, see README).
"""
import os
import sys

from repo_paths import RESULT_DIR
import batch_common as bc

SUMMARY_ONLY = False

# Largest Sioux Falls instance to run (the paper reports 1 to 16; instance files exist for 1 to 24).
# The 9-node network always runs 1 to 9.
MAX_SF = 16

LOG_DIR = os.path.join(bc.CODE_DIR, 'run_logs')
ACC_RESULT_DIR = os.path.join(RESULT_DIR, 'Performance of acceleration components')
SUMMARY_CSV = os.path.join(RESULT_DIR, 'metrics_summary.csv')


def main():
    jobs = bc.plan(MAX_SF)
    if '--list' in sys.argv:
        for i, s in enumerate(jobs, 1):
            print('%2d. %s' % (i, s))
        return
    if SUMMARY_ONLY or '--summary' in sys.argv:
        bc.collect(ACC_RESULT_DIR, SUMMARY_CSV)
        return
    bc.run_jobs(jobs, bc.DRIVER_DIR, LOG_DIR)
    bc.collect(ACC_RESULT_DIR, SUMMARY_CSV)


if __name__ == '__main__':
    main()
