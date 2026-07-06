#!/usr/bin/env python3
# coding=utf-8
"""
机械臂控制模块 - 机器狗2 四足机器人
=============================================
集成版本（单文件），包含：
- 配置参数、串口通信安全、舵机补偿
- 安全初始化/关机、抓取、搬运、放置全流程
- 异常处理、低电量自适应、视觉辅助

适用型号：XGO-Lite（树莓派 CM4）
模拟运行：Windows/Linux PC 上自动使用 Mock 硬件（无需树莓派）

运行方式：
    # 树莓派上
    sudo python3 arm_control.py --zone=A
    # Windows/PC 上（模拟模式）
    python arm_control.py --sim --zone=A
    python arm_control.py --sim --test-grasp
    python arm_control.py --sim --test-arm

视觉模块接口（接收外部数据）：
    arm_control.set_ball_data(distance, angle_x, angle_y)
    arm_control.set_marker_data(offset_x, offset_y)
"""