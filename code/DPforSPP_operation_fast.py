# -*- coding: utf-8 -*-
# Thin wrapper: DPforSPP_operation interface over DPforSPP_fast (B&P-DP).
# Original file name in the authors' project: DPforSPP_operation_fast.py
"""DPforSPP_operation 的加速替身（不含有效不等式），签名与返回值完全一致。"""
import DPforSPP_fast as _fast


def shortest_path_operations(operations, network, truckGroupCapacity):
    return _fast.shortest_path_operations_nocut(operations, network, truckGroupCapacity)
