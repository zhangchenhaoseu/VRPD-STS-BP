# -*- coding: utf-8 -*-
"""Package the instance data and the result files of the original project into the zip archives distributed
with this repository (run once, on the machine that holds the original project folder).

    python tools/collect_data.py --src "<original project folder>" [--dst <repository root>]

The original folder is the one that contains code/, instanceData/, result/, cap30/, abs/ and the three
"...可达性下五组instanceData..." folders. The archives are written to <dst>/data_archives/ and contain
repository-relative paths, so that tools/unpack_data.py restores them next to code/:

    instanceData_9node.zip                   instanceData/<9nodeNetwork_* and timeDependent_9nodeNetwork_*>/
    instanceData_SF_time_independent.zip     instanceData/SFnetwork_<1..24>customer/
    instanceData_SF_time_dependent.zip       instanceData/timeDependent_SFnetwork_<1..24>customer/
    instanceData_sensitivity_current.zip     instanceData/敏感性分析_*/ as last left in the project, the initial solutions of
                                             the sensitivity customer sets, and the notes/time-window files of instanceData/
    instanceData_abs.zip                     abs/instanceData/<instance>/        (rendezvous-restricted instances, Table 12)
    results_main.zip                         result/Performance of acceleration components/<instance>/  (iniOperation.txt,
                                             metrics(*).json, result(*).txt), result/Performance of the proposed algorithm/
                                             (Gurobi MIP and truck-only results) and result/metrics_summary*.csv
    results_cap25.zip                        result/Performance of acceleration components (cap25)/      (run_cap.py output)
    results_cap30.zip                        cap30/result/                                               (setup_cap.py output)
    results_abs.zip                          abs/result/                                                 (setup_abs.py output)
    sensitivity_rendezvous_depot.zip         sensitivity_rendezvous/depot/{instanceData,result/...}/<instance>/
    sensitivity_rendezvous_customer.zip      the same for the customer-set rankings
    sensitivity_rendezvous_network.zip       the same for the transportation-network rankings
Archives larger than about 80 MB are split into _part1, _part2, ... (GitHub limits single files to 100 MB).
Caches, logs and IDE folders are skipped. Nothing in the source folder is modified.
"""
import argparse
import os
import sys
import zipfile

SKIP_DIRS = {'__pycache__', '.idea', 'run_logs', 'run_logs_cap25', '_probe_可删除'}
SKIP_FILES = {'probe.txt', 'Thumbs.db', '.DS_Store'}
PART_LIMIT = 80 * 1024 * 1024


class Archive:
    """A zip archive that is split into parts when it grows beyond PART_LIMIT."""

    def __init__(self, dst_dir, name):
        self.dst_dir, self.name, self.part, self.zf, self.count = dst_dir, name, 0, None, 0
        self.paths = []

    def _open(self):
        self.part += 1
        p = os.path.join(self.dst_dir, '%s.zip' % self.name if self.part == 1 else '%s_part%d.zip' % (self.name, self.part))
        self.paths.append(p)
        self.zf = zipfile.ZipFile(p, 'w', zipfile.ZIP_DEFLATED, compresslevel=9)

    def add_tree(self, src_dir, arc_prefix):
        """Add one folder (recursively) under arc_prefix; start a new part first if the current one is big."""
        if self.zf is None:
            self._open()
        elif self.zf.fp.tell() > PART_LIMIT:        # bytes written so far to the current part
            self.zf.close()
            self._open()
        for root, dirs, files in os.walk(src_dir):
            dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS)
            for f in sorted(files):
                if f in SKIP_FILES:
                    continue
                full = os.path.join(root, f)
                rel = os.path.relpath(full, src_dir).replace(os.sep, '/')
                self.zf.write(full, arc_prefix + '/' + rel)
                self.count += 1

    def add_file(self, src_file, arc_name):
        if self.zf is None:
            self._open()
        self.zf.write(src_file, arc_name)
        self.count += 1

    def close(self):
        if self.zf is not None:
            self.zf.close()
        if self.count == 0:                      # nothing to archive: do not leave an empty zip behind
            for p in self.paths:
                os.remove(p)
            self.paths = []
            return 0
        if self.part == 1 and os.path.getsize(self.paths[0]) > PART_LIMIT:
            print('  warning: %s is larger than the part limit; split it by hand if GitHub refuses it' % self.paths[0])
        return self.count


def instance_dirs(folder, pred):
    if not os.path.isdir(folder):
        return []
    return sorted(d for d in os.listdir(folder) if os.path.isdir(os.path.join(folder, d)) and d not in SKIP_DIRS and pred(d))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--src', required=True, help='original project folder (contains code/, instanceData/, result/ ...)')
    ap.add_argument('--dst', default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))), help='repository root (default: parent of tools/)')
    a = ap.parse_args()
    src, dst = a.src, os.path.join(a.dst, 'data_archives')
    if not os.path.isdir(os.path.join(src, 'instanceData')):
        print('instanceData/ not found under', src)
        return 1
    os.makedirs(dst, exist_ok=True)
    summary = []

    def tree_archive(name, items):
        """items: list of (source folder, archive prefix)"""
        ar = Archive(dst, name)
        n_dirs = 0
        for folder, prefix in items:
            if os.path.isdir(folder):
                ar.add_tree(folder, prefix)
                n_dirs += 1
            else:
                print('  (missing, skipped) %s' % folder)
        n = ar.close()
        summary.append((name, n_dirs, n, [os.path.getsize(p) for p in ar.paths]))

    idata = os.path.join(src, 'instanceData')
    is_nine = lambda d: d.startswith('9nodeNetwork_') or d.startswith('timeDependent_9nodeNetwork_')
    is_sf_ti = lambda d: d.startswith('SFnetwork_')
    is_sf_td = lambda d: d.startswith('timeDependent_SFnetwork_')
    is_sens = lambda d: d.startswith('敏感性分析')

    print('instance data ...')
    tree_archive('instanceData_9node', [(os.path.join(idata, d), 'instanceData/' + d) for d in instance_dirs(idata, is_nine)])
    tree_archive('instanceData_SF_time_independent', [(os.path.join(idata, d), 'instanceData/' + d) for d in instance_dirs(idata, is_sf_ti)])
    tree_archive('instanceData_SF_time_dependent', [(os.path.join(idata, d), 'instanceData/' + d) for d in instance_dirs(idata, is_sf_td)])
    ar = Archive(dst, 'instanceData_sensitivity_current')
    nd = 0
    for d in instance_dirs(idata, is_sens):
        ar.add_tree(os.path.join(idata, d), 'instanceData/' + d)
        nd += 1
    for f in sorted(os.listdir(idata)):          # notes and time-window records kept next to the instance folders
        p = os.path.join(idata, f)
        if os.path.isfile(p) and f not in SKIP_FILES:
            ar.add_file(p, 'instanceData/_notes/' + f)
    for f in ('敏感性分析结果.xlsx', '实例计算初始解记录.txt'):
        p = os.path.join(src, f)
        if os.path.isfile(p):
            ar.add_file(p, 'instanceData/_notes/' + f)
    summary.append(('instanceData_sensitivity_current', nd, ar.close(), [os.path.getsize(p) for p in ar.paths]))
    abs_data = os.path.join(src, 'abs', 'instanceData')
    tree_archive('instanceData_abs', [(os.path.join(abs_data, d), 'abs/instanceData/' + d) for d in instance_dirs(abs_data, lambda d: True)])

    print('results ...')
    res = os.path.join(src, 'result')
    acc = os.path.join(res, 'Performance of acceleration components')
    alg = os.path.join(res, 'Performance of the proposed algorithm')
    ar = Archive(dst, 'results_main')
    nd = 0
    for folder, prefix in ((acc, 'result/Performance of acceleration components'), (alg, 'result/Performance of the proposed algorithm')):
        for d in instance_dirs(folder, lambda d: True):
            ar.add_tree(os.path.join(folder, d), prefix + '/' + d)
            nd += 1
        for f in sorted(os.listdir(folder)) if os.path.isdir(folder) else []:
            p = os.path.join(folder, f)
            if os.path.isfile(p) and f.endswith('.txt'):
                ar.add_file(p, prefix + '/' + f)
    for f in sorted(os.listdir(res)):
        if f.startswith('metrics_summary') and f.endswith('.csv'):
            ar.add_file(os.path.join(res, f), 'result/' + f)
    summary.append(('results_main', nd, ar.close(), [os.path.getsize(p) for p in ar.paths]))
    cap25 = os.path.join(res, 'Performance of acceleration components (cap25)')
    tree_archive('results_cap25', [(os.path.join(cap25, d), 'result/Performance of acceleration components (cap25)/' + d) for d in instance_dirs(cap25, lambda d: True)])
    cap30 = os.path.join(src, 'cap30', 'result')
    ar = Archive(dst, 'results_cap30')
    nd = 0
    c30 = os.path.join(cap30, 'Performance of acceleration components')
    for d in instance_dirs(c30, lambda d: True):
        ar.add_tree(os.path.join(c30, d), 'cap30/result/Performance of acceleration components/' + d)
        nd += 1
    for f in sorted(os.listdir(cap30)) if os.path.isdir(cap30) else []:
        if f.endswith('.csv'):
            ar.add_file(os.path.join(cap30, f), 'cap30/result/' + f)
    summary.append(('results_cap30', nd, ar.close(), [os.path.getsize(p) for p in ar.paths]))
    absr = os.path.join(src, 'abs', 'result')
    ar = Archive(dst, 'results_abs')
    nd = 0
    ab = os.path.join(absr, 'Performance of acceleration components')
    for d in instance_dirs(ab, lambda d: True):
        ar.add_tree(os.path.join(ab, d), 'abs/result/Performance of acceleration components/' + d)
        nd += 1
    for f in sorted(os.listdir(absr)) if os.path.isdir(absr) else []:
        if f.endswith('.csv'):
            ar.add_file(os.path.join(absr, f), 'abs/result/' + f)
    summary.append(('results_abs', nd, ar.close(), [os.path.getsize(p) for p in ar.paths]))

    print('rendezvous-distribution experiment (Section 6.4.3) ...')
    for tag, zh in (('depot', '车场'), ('customer', '客户'), ('network', '网络')):
        hist = os.path.join(src, '%s可达性下五组instanceData历史数据' % zh)
        ops = os.path.join(src, '%s可达性下五组instanceData操作&结果' % zh)
        items = [(os.path.join(hist, d), 'sensitivity_rendezvous/%s/instanceData/%s' % (tag, d)) for d in instance_dirs(hist, lambda d: True)]
        items += [(os.path.join(ops, d), 'sensitivity_rendezvous/%s/result/Performance of acceleration components/%s' % (tag, d)) for d in instance_dirs(ops, lambda d: True)]
        tree_archive('sensitivity_rendezvous_' + tag, items)

    print('\narchives written to %s' % dst)
    for name, nd, n, sizes in summary:
        print('  %-38s %3d folders %6d files  %s' % (name, nd, n, ', '.join('%.1f MB' % (s / 1048576.0) for s in sizes)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
