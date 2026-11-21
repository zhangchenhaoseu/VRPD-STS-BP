# -*- coding: utf-8 -*-
# Optional: redirect the verbose console output of a driver to a log file (import quiet).
# Original file name in the authors' project: quiet.py
"""
把驱动脚本的刷屏输出改写到日志文件，控制台只保留关键行。

原因：列生成每一轮、每个子问题都会 print 一次乘子字典，其中
capacityPi_DICT 在 12 客户时有 4095 个条目。在 PyCharm 的 Run 窗口里，
渲染这种超长输出本身就很花时间，小算例上甚至比计算还慢。

用法：在驱动脚本的 import 区最后加一行

    import quiet

即可。日志写到 code\run_console.log，程序结束时控制台会提示路径。
不想用就把这一行注释掉，输出恢复原样。
"""

import atexit
import io
import os
import sys

LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'run_console.log')

# 以这些前缀开头的行仍然直接显示在控制台
PASSTHROUGH = ('[metrics]', '!!', '目标值', '分支定价求解时间', 'RMP目标函数值')


class _Tee(object):
    """默认写日志；命中白名单前缀的行同时回显到控制台。"""

    def __init__(self, log, console):
        self._log = log
        self._console = console
        self._buf = ''

    def write(self, s):
        self._log.write(s)
        self._buf += s
        while '\n' in self._buf:
            line, self._buf = self._buf.split('\n', 1)
            if line.startswith(PASSTHROUGH):
                self._console.write(line + '\n')

    def flush(self):
        try:
            self._log.flush()
            self._console.flush()
        except Exception:
            pass


_console = sys.stdout
_log = io.open(LOG_PATH, 'w', encoding='utf-8', buffering=1 << 20)
sys.stdout = _Tee(_log, _console)


@atexit.register
def _restore():
    try:
        sys.stdout = _console
        _log.flush()
        _log.close()
        _console.write('[quiet] 完整控制台输出已写入 %s\n' % LOG_PATH)
    except Exception:
        pass
