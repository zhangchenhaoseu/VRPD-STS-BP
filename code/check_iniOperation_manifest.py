# -*- coding: utf-8 -*-
"""Integrity check of the Sioux Falls instances and of their operation files, and (optionally) reset of the
run_all.py completion markers so that the instances are re-run.

The check verifies that
  1. the 48 Sioux Falls instances (1 to 24 customers, both settings) in DATA_DIR have time windows of width 15
     (STS_node_spaceAndTime.txt);
  2. the 48 operation files result/Performance of acceleration components/<instance>/iniOperation.txt are present and
     have the row count and MD5 recorded in MANIFEST below (the files built with constructOperation_fast.py for the
     width-15 windows, which were used for all reported Sioux Falls runs).
With --reset (or RESET = True) it then deletes the .done and .cutsdone markers of these drivers in code/run_logs/
(and, with --with-9node, those of the 18 drivers of the 9-node network) so that run_all.py runs them again.
Nothing is deleted when a check fails.

Usage:
    python check_iniOperation_manifest.py                # check only
    python check_iniOperation_manifest.py --reset        # check, then delete the markers of the 48 SF drivers
    python check_iniOperation_manifest.py --reset --with-9node

Original file: prepare_tw15_rerun.py (the MANIFEST is copied verbatim).
"""
import csv
import hashlib
import io
import os
import sys

from repo_paths import DATA_DIR, RESULT_DIR

CODE_DIR = os.path.dirname(os.path.abspath(__file__))
ACC_RESULT_DIR = os.path.join(RESULT_DIR, 'Performance of acceleration components')
LOG_DIR = os.path.join(CODE_DIR, 'run_logs')

WIDTH = 15
MAX_SF = 24

RESET = False          # True: delete the completion markers after a successful check
WITH_9NODE = False     # True: also the markers of the 18 drivers of the 9-node network

# 48 iniOperation.txt files: (number of rows without the header, md5 of the file with LF line endings)
MANIFEST = {
    'SFnetwork_1customer': (1620, '1975d2578cf88e3701df9c2ef9bfad0d'),
    'SFnetwork_2customer': (3194, 'edb836a8579effa0f7e0c71a2025bbc1'),
    'SFnetwork_3customer': (4828, '3f6f2b43a145f5d2008e2c5d35ddbb25'),
    'SFnetwork_4customer': (6447, 'ddc80882f831ce70e82197aa29c561ff'),
    'SFnetwork_5customer': (7043, 'c1bc830adc96907f0449455301d3e21e'),
    'SFnetwork_6customer': (8092, '76f510b2128576562f2458e9ce9bc1f5'),
    'SFnetwork_7customer': (8769, 'cdc5d8578780647df5cd8c701e509c02'),
    'SFnetwork_8customer': (12260, '5c2983dbad305036ccb5a450fc57c668'),
    'SFnetwork_9customer': (13265, '1d3a1a19760ffc052db9bc47d27b879c'),
    'SFnetwork_10customer': (17863, 'ec03720bc45dc7187209f28225e44640'),
    'SFnetwork_11customer': (20100, 'c6a4921a316343c3c80ad3e7150b4ade'),
    'SFnetwork_12customer': (25236, 'ed5f0bfc557fd1e01eb37f30af83c20b'),
    'SFnetwork_13customer': (27442, 'dbd08216e1ea16042f5e128fa5cdae8f'),
    'SFnetwork_14customer': (28207, '9ae01365ed4237abe2ebc93a81a5d0be'),
    'SFnetwork_15customer': (31090, '6e755ad2a7db94306378ee4bbc1d38bb'),
    'SFnetwork_16customer': (34585, 'fc9380299d21f8cf6723a42cf0ac1209'),
    'SFnetwork_17customer': (38824, '30f0631b64f18e5b8f79963a78bf0d9f'),
    'SFnetwork_18customer': (38751, '213fd298c81a0bf5998e8e4745355151'),
    'SFnetwork_19customer': (41381, 'e45add9a98f738066c98458868bcc982'),
    'SFnetwork_20customer': (42110, 'c86a6ea9c903d8d9bb273a9fe272d8d2'),
    'SFnetwork_21customer': (42828, '508cc54b1d3490c924ec175fbed1e104'),
    'SFnetwork_22customer': (48047, '5d484df7333969aac02d1c093ff9b005'),
    'SFnetwork_23customer': (52261, '18312711ba41a5fec784d53c65d29a49'),
    'SFnetwork_24customer': (55734, '6d574f099f7a6b9857b1791b5ba7dc3e'),
    'timeDependent_SFnetwork_1customer': (1725, 'c06c6c57b43b6981f4b2aaa01622a199'),
    'timeDependent_SFnetwork_2customer': (3781, '2c9d0d14ab9c98bb480d1db4812695c0'),
    'timeDependent_SFnetwork_3customer': (5589, 'c0162600f9a7e3b1742d5cadcde451b8'),
    'timeDependent_SFnetwork_4customer': (7532, 'c82b32ddcc557e48f23a4fac2b41cc81'),
    'timeDependent_SFnetwork_5customer': (7761, 'ef7f23861c475f58ac7dbbe5d696fb45'),
    'timeDependent_SFnetwork_6customer': (8644, '9f7ad45599203e1de82c29dde2ae4a56'),
    'timeDependent_SFnetwork_7customer': (8871, 'd45e98da244e7d2bb13b8db10960f028'),
    'timeDependent_SFnetwork_8customer': (9837, '7fdf5f9b72acff32ec1f3d719ed5942b'),
    'timeDependent_SFnetwork_9customer': (10121, 'c3372df2001a4ac98054886478e77024'),
    'timeDependent_SFnetwork_10customer': (11242, '4f48afe14e556f4b1d0bea08acc61a57'),
    'timeDependent_SFnetwork_11customer': (11765, '3a6168c12794db582a54777451b6717d'),
    'timeDependent_SFnetwork_12customer': (13847, '71a67016bdcd32f5faf2e944f3710924'),
    'timeDependent_SFnetwork_13customer': (15051, '0702e686b05481c3a33a3d49bb800d38'),
    'timeDependent_SFnetwork_14customer': (15205, 'a8b4ccc89df9ee59e08e0acaca5f7148'),
    'timeDependent_SFnetwork_15customer': (15777, '6205de611ddc28e35eed1b351dc8f5ce'),
    'timeDependent_SFnetwork_16customer': (16642, 'f903a10880ab0fd14684afb67a39a8d2'),
    'timeDependent_SFnetwork_17customer': (18468, 'da1830c9747802323e6597cc527512da'),
    'timeDependent_SFnetwork_18customer': (20407, 'c6690a3300e3329757170df16b8b37f0'),
    'timeDependent_SFnetwork_19customer': (21747, '759e48a8998ff9c0bbed8719a4c2cc37'),
    'timeDependent_SFnetwork_20customer': (22277, '015ebdc8c352afd34b3683087581a754'),
    'timeDependent_SFnetwork_21customer': (22592, '042f63802c69f94cf93771e904f495fa'),
    'timeDependent_SFnetwork_22customer': (23492, '0926221ca133c6688ce583c6e2bf1086'),
    'timeDependent_SFnetwork_23customer': (24795, '433ac337598d8a9af66eba0489bcd7e1'),
    'timeDependent_SFnetwork_24customer': (25660, 'bf6af3ed8dd936b3a26df3d28b24bce7'),
}


def sf_instances():
    for pre in ('', 'timeDependent_'):
        for n in range(1, MAX_SF + 1):
            yield '%sSFnetwork_%dcustomer' % (pre, n)


def driver_base(name):
    td = name.startswith('timeDependent_')
    n = int(name.split('SFnetwork_')[1].split('customer')[0])
    return 'bp_SF%s_%d' % ('_td' if td else '', n)


def window_widths(name):
    f = os.path.join(DATA_DIR, name, 'STS_node_spaceAndTime.txt')
    if not os.path.exists(f):
        return None
    got = {}
    with io.open(f, encoding='utf-8') as fh:
        for r in csv.DictReader(fh):
            if int(r['timeWindow']) == 1:
                got.setdefault(int(r['space']), []).append(int(r['time']))
    ids = sorted(k for k in got if k >= 25)
    if len(ids) > 1:
        ids = ids[:-1]                      # the last one is the end depot
    return [max(got[c]) - min(got[c]) for c in ids]


def ops_fingerprint(name):
    f = os.path.join(ACC_RESULT_DIR, name, 'iniOperation.txt')
    if not os.path.exists(f):
        return None
    h = hashlib.md5()
    rows = -1
    with open(f, 'rb') as fh:
        for line in fh:
            h.update(line.replace(b'\r\n', b'\n'))
            rows += 1
    return rows, h.hexdigest()


def main():
    reset = RESET or '--reset' in sys.argv
    with9 = WITH_9NODE or '--with-9node' in sys.argv
    bad = []
    names = list(sf_instances())
    for name in names:
        w = window_widths(name)
        if w is None:
            bad.append((name, 'instance folder or STS_node_spaceAndTime.txt missing in DATA_DIR'))
        elif max(w) > WIDTH:
            bad.append((name, 'widest time window is %d, not the width-%d data' % (max(w), WIDTH)))
    for name in names:
        fp = ops_fingerprint(name)
        exp = MANIFEST.get(name)
        if exp is None:
            bad.append((name, 'not in the manifest'))
        elif fp is None:
            bad.append((name, 'iniOperation.txt missing'))
        elif fp != tuple(exp):
            bad.append((name, 'iniOperation.txt differs from the manifest: %d rows md5 %s, expected %d rows md5 %s'
                        % (fp[0], fp[1][:8], exp[0], exp[1][:8])))
    if bad:
        print('check failed (%d problems), nothing deleted:' % len(bad))
        for name, why in bad:
            print('   %-40s %s' % (name, why))
        return 1
    print('check passed: %d instances with time windows of width %d, %d operation files match the manifest.'
          % (len(names), WIDTH, len(names)))
    if not reset:
        return 0
    bases = [driver_base(name) for name in names]
    if with9:
        for pre in ('', '_td'):
            for n in range(1, 10):
                bases.append('bp_9node%s_%d' % (pre, n))
    todo = []
    for base in bases:
        for ext in ('.done', '.cutsdone'):
            p = os.path.join(LOG_DIR, base + ext)
            if os.path.exists(p):
                todo.append(p)
    for p in todo:
        os.remove(p)
    print('%d completion markers deleted; run_all.py will run these drivers again.' % len(todo))
    return 0


if __name__ == '__main__':
    sys.exit(main())
