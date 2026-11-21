# -*- coding: utf-8 -*-
# Process pool shared by the parallel pricing; the network object is sent to the workers once.
# Original file name in the authors' project: mppool.py
"""
全局复用一个进程池，并把静态的网络对象在建池时一次性发给工作进程。

原实现有三处开销：
  1. columnGenerationDP_CUT_PA 开头 pool = mp.Pool(...) 创建后从未关闭，
     每进一次列生成（即每个分支节点）泄漏一个进程池；
  2. 列生成循环里又用 with mp.Pool(...) 每一轮重新创建一次；
  3. 每个 pool.map 任务的负载里都带着整个 network 对象（空间-时间-状态
     节点、车辆弧、无人机弧、潜在架次），每轮每个子问题都要完整序列化、
     传输、再反序列化一遍。子问题本身加速之后，这部分反而成了主要耗时。

本模块把进程池收敛为整个进程内的一个，并用 initializer 把 network 只发
一次；此后任务负载里的 network 位置传 None，工作进程自己取本地那一份。
network 在整个求解过程中不变，所以这样做是安全的。
"""

import atexit
import multiprocessing as mp

_POOL = None
_N = None

_WORKER_NET = None      # 只在工作进程里有值


def _init_worker(net):
    global _WORKER_NET
    _WORKER_NET = net


def worker_network():
    """工作进程侧调用：取建池时下发的 network。"""
    return _WORKER_NET


def get_pool(n, network=None):
    global _POOL, _N
    if _POOL is None or _N != n:
        close()
        if network is None:
            _POOL = mp.Pool(processes=n)
        else:
            _POOL = mp.Pool(processes=n, initializer=_init_worker, initargs=(network,))
        _N = n
    return _POOL


def close():
    global _POOL, _N
    if _POOL is not None:
        try:
            _POOL.close()
            _POOL.join()
        except Exception:
            pass
    _POOL, _N = None, None


atexit.register(close)
