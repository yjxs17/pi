#!/usr/bin/env python3
# coding=utf-8
"""
XGO-Lite 四足机器人舞蹈程序
设计者：冰零❄️（创意）+ 冰灵🔧（技术可行性）
调度：冰淇淋🍦

适用型号：XGO-Lite（树莓派 CM4）
"""

import os
import sys
import time

# 树莓派串口权限
os.system("sudo chmod 777 -R /dev/ttyAMA0")

# 导入XGO库（仅在树莓派上可用）
from xgolib import XGO

# ============================================================================
# 舞蹈参数配置
# ============================================================================
BPM = 120
BEAT_DURATION = 60.0 / BPM