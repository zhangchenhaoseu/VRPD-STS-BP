# -*- coding: utf-8 -*-
"""Helpers shared by the batch runners (run_all.py, run_cuts.py, run_cap.py and the runners
written by setup_cap.py / setup_abs.py): driver file names, job plans, running one driver in a
sub-process with a trimmed log, and collecting the metrics(*).json files into one CSV.

The original project had these helpers duplicated inside each runner; the logic is unchanged,
only the driver file names (bp_<network>[_td]_<n>.py in code/drivers/) are new.
"""
import csv
import io
import json
import os
import subprocess
import sys
import time

CODE_DIR = os.path.dirname(os.path.abspath(__file__))
DRIVER_DIR = os.path.join(CODE_DIR, 'drivers')

# Logs keep only the head and the tail of the console output (one time-dependent SF run once wrote
# a 19 GB log).
HEAD_BYTES = 64 * 1024
TAIL_BYTES = 256 * 1024

FIELDS = ['instance', 'algorithm', 'UB', 'LB', 'gap', 'proved_optimal',
          'bb_nodes', 'cg_iters', 'pricing_calls',
          'SP_time', 'RMP_time', 'total_time',
          'pricing_time_total', 'pricing_time_mean', 'pricing_time_max', 'pricing_time_min',
          'root_lp', 'n_rci_enumerated', 'n_rci_active_root']


def script_name(network, n, td):
    """Driver file for an instance: network 'nine' or 'sf', n customers, td = time-dependent."""
    return 'bp_%s%s_%d.py' % ('9node' if network == 'nine' else 'SF', '_td' if td else '', n)


def instance_name(network, n, td):
    return '%s%s_%dcustomer' % ('timeDependent_' if td else '', '9nodeNetwork' if network == 'nine' else 'SFnetwork', n)


def plan(max_sf, driver_dir=DRIVER_DIR, with_nine=True, argv=None):
    """Jobs ordered by expected cost: 9-node network first, then Sioux Falls; for each size the
    time-independent instance before the time-dependent one. --only=SF_td_11,SF_12 keeps only the
    drivers whose file name contains one of the comma-separated sub-strings."""
    argv = sys.argv if argv is None else argv
    jobs = []
    if with_nine:
        for n in range(1, 10):
            for td in (False, True):
                jobs.append(script_name('nine', n, td))
    for n in range(1, max_sf + 1):
        for td in (False, True):
            jobs.append(script_name('sf', n, td))
    jobs = [j for j in jobs if os.path.exists(os.path.join(driver_dir, j))]
    only = [a[len('--only='):] for a in argv if a.startswith('--only=')]
    if only:
        keys = [k for s in only for k in s.split(',') if k]
        jobs = [j for j in jobs if any(k in j for k in keys)]
    return jobs


def collect(result_dir, summary_csv):
    """Gather every metrics(*).json below result_dir into summary_csv (one row per instance and
    algorithm). Negative gaps caused by floating point noise are reported as 0."""
    rows = []
    if not os.path.isdir(result_dir):
        print('[collect] result directory not found:', result_dir)
        return
    for inst in sorted(os.listdir(result_dir)):
        d = os.path.join(result_dir, inst)
        if not os.path.isdir(d):
            continue
        for fn in sorted(os.listdir(d)):
            if not (fn.startswith('metrics(') and fn.endswith('.json')):
                continue
            try:
                with io.open(os.path.join(d, fn), encoding='utf-8') as fh:
                    j = json.load(fh)
            except Exception as e:
                print('[collect] cannot read %s/%s: %s' % (inst, fn, e))
                continue
            row = {k: j.get(k, '') for k in FIELDS}
            try:
                if row['gap'] not in ('', None) and float(row['gap']) < 0:
                    row['gap'] = 0.0
            except (TypeError, ValueError):
                pass
            rows.append(row)
    rows.sort(key=lambda r: (str(r['instance']), str(r['algorithm'])))
    os.makedirs(os.path.dirname(summary_csv), exist_ok=True)
    with io.open(summary_csv, 'w', encoding='utf-8-sig', newline='') as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)
    print('[collect] %d rows written to %s' % (len(rows), summary_csv))


def run_one(script_path, log, cwd, env_extra=None):
    """Run one driver; its console output goes straight to the log file (readable while the run is
    in progress). Returns (return code, number of bytes written)."""
    env = dict(os.environ, PYTHONHASHSEED='0', PYTHONUNBUFFERED='1')  # fixed hash seed; unbuffered log
    if env_extra:
        env.update(env_extra)
    with open(log, 'wb') as fh:
        p = subprocess.Popen([sys.executable, script_path], cwd=cwd, env=env, stdout=fh, stderr=subprocess.STDOUT)
        p.wait()
    total = os.path.getsize(log)
    if total > HEAD_BYTES + TAIL_BYTES:
        with open(log, 'rb') as fh:
            head = fh.read(HEAD_BYTES)
            fh.seek(total - TAIL_BYTES)
            tail = fh.read()
        with open(log, 'wb') as fh:
            fh.write(head)
            fh.write(('\n\n...... about %.1f MB of output omitted ......\n\n'
                      % ((total - HEAD_BYTES - TAIL_BYTES) / 1048576.0)).encode('utf-8'))
            fh.write(tail)
    return p.returncode, total


def run_jobs(jobs, driver_dir, log_dir, done_suffix='.done', log_suffix='.log', env_extra=None, label=''):
    """Run the jobs one after another, skipping those with a completion marker in log_dir."""
    if not os.path.isdir(log_dir):
        os.makedirs(log_dir)
    print('%s%d drivers, logs in %s\n' % (label, len(jobs), log_dir))
    t_all = time.time()
    for i, s in enumerate(jobs, 1):
        base = os.path.splitext(s)[0]
        done = os.path.join(log_dir, base + done_suffix)
        log = os.path.join(log_dir, base + log_suffix)
        if os.path.exists(done):
            print('[%2d/%d] skipped (done)   %s' % (i, len(jobs), s))
            continue
        print('[%2d/%d] running %s' % (i, len(jobs), s), end='', flush=True)
        t0 = time.time()
        rc, nbytes = run_one(os.path.join(driver_dir, s), log, cwd=driver_dir, env_extra=env_extra)
        dt = time.time() - t0
        if rc == 0:
            with io.open(done, 'w', encoding='utf-8') as fh:
                fh.write('%.1f s\n' % dt)
            print('  finished in %.1f s (%.1f MB of output)' % (dt, nbytes / 1048576.0))
        else:
            print('  [FAILED] exit code %d, see %s' % (rc, log))
    print('\nall done, total %.1f s' % (time.time() - t_all))
