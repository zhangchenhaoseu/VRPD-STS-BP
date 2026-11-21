# -*- coding: utf-8 -*-
"""Extract the data archives of data_archives/ into the repository (the inverse of collect_data.py).

    python tools/unpack_data.py              # extract every archive
    python tools/unpack_data.py --list       # only list the archives and their top-level folders
    python tools/unpack_data.py instanceData_9node results_main     # only these archives (name without .zip / _partN)

The archives contain repository-relative paths (instanceData/..., result/..., cap30/..., abs/..., sensitivity_rendezvous/...),
so after extraction the drivers find their data with the default repo_paths.py settings. Existing files are overwritten.
"""
import os
import sys
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARCH = os.path.join(ROOT, 'data_archives')


def main():
    if not os.path.isdir(ARCH):
        print('data_archives/ not found; run tools/collect_data.py on the original project first or download the archives')
        return 1
    zips = sorted(f for f in os.listdir(ARCH) if f.endswith('.zip'))
    wanted = [a for a in sys.argv[1:] if not a.startswith('--')]
    if wanted:
        zips = [z for z in zips if any(z == w + '.zip' or z.startswith(w + '_part') for w in wanted)]
    if '--list' in sys.argv:
        for z in zips:
            with zipfile.ZipFile(os.path.join(ARCH, z)) as zf:
                tops = sorted({n.split('/')[0] + '/' + n.split('/')[1] for n in zf.namelist() if n.count('/') >= 1})
                print('%-45s %6d files  %s' % (z, len(zf.namelist()), ', '.join(tops[:3]) + (' ...' if len(tops) > 3 else '')))
        return 0
    for z in zips:
        with zipfile.ZipFile(os.path.join(ARCH, z)) as zf:
            names = zf.namelist()
            bad = [n for n in names if n.startswith('/') or '..' in n.split('/')]
            if bad:
                print('refusing %s: unsafe paths %s' % (z, bad[:3]))
                continue
            zf.extractall(ROOT)
            print('%-45s %6d files extracted' % (z, len(names)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
