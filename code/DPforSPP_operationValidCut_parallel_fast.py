# -*- coding: utf-8 -*-
# Thin wrapper: parallel interface over DPforSPP_fast, network taken from the worker process (B&P-DP-VI-P).
# Original file name in the authors' project: DPforSPP_operationValidCut_parallel_fast.py
"""DPforSPP_operationValidCut_parallel 的加速替身，签名与返回值完全一致。

任务负载里的 network 允许为 None，此时改用建池时下发给本工作进程的那一份，
避免每一轮把整个网络对象重新传一遍。
"""
import DPforSPP_fast as _fast


def shortest_path_operations(input_data):
    operations, network, truckGroupCapacity, capacityPi_DICT = input_data
    if network is None:
        import mppool
        network = mppool.worker_network()
        if network is None:
            raise RuntimeError('工作进程没有拿到 network，请检查 mppool.get_pool 是否传入了 network')
    return _fast.shortest_path_operations_parallel(
        [operations, network, truckGroupCapacity, capacityPi_DICT])
