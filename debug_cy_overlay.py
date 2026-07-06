"""
debug_cy_overlay.py — 实时显示球位置，cy 直接画在画面上
- 弹窗实时显示摄像头画面 + 球的位置(cx, cy, area)
- 按 空格键：终端打印当前 cy 值
- 按 s 键：保存当前帧到 ~/debug_cy_x.jpg
- 按 q 键：退出

用法：在树莓派桌面终端运行  python3 debug_cy_overlay.py
"""