# -*- coding: utf-8 -*-
# Thin wrapper: DPforSPP_operationValidCut interface over DPforSPP_fast (B&P-DP-VI).
# Original file name in the authors' project: DPforSPP_operationValidCut_fast.py
"""DPforSPP_operationValidCut 的加速替身，签名与返回值完全一致。"""
import DPforSPP_fast as _fast


def shortest_path_operations(operations, network, truckGroupCapacity, capacityPi_DICT):
    return _fast.shortest_path_operations(operations, network, truckGroupCapacity,
                                          capacityPi_DICT)
