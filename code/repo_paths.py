# -*- coding: utf-8 -*-
"""Repository-relative paths shared by every script (replaces the absolute Windows paths of the
original project).

ROOT        repository root (the parent of code/)
DATA_DIR    instance folders, ROOT/instanceData by default
RESULT_DIR  result folders,   ROOT/result by default

Both directories can be redirected with the environment variables VRPD_DATA_DIR and
VRPD_RESULT_DIR, which is how the isolated capacity (cap25/cap30) and rendezvous-restricted
(abs) experiments are run without touching the main results.

Importing this module also puts code/ and code/networks/ on sys.path, so that the scripts in
the sub-folders can import the shared modules and the per-instance network modules by name.
"""
import os
import sys

CODE_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(CODE_DIR)
DATA_DIR = os.environ.get('VRPD_DATA_DIR') or os.path.join(ROOT, 'instanceData')
RESULT_DIR = os.environ.get('VRPD_RESULT_DIR') or os.path.join(ROOT, 'result')

for _p in (os.path.join(CODE_DIR, 'networks'), CODE_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)
