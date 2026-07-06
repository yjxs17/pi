#!/usr/bin/env python3
# coding=utf-8
"""
calibrate_distance.py - 距离标定工具
========================================
目的：测量不同距离下球的 cy 值，生成距离-cy 对应关系
输出：标定数据保存到 distance_calibration.csv

使用步骤：
1. 把球放在狗前面 30cm 处
2. 按空格键记录当前 cy 值
3. 逐次移动球到 40、50、60...100cm
4. 按 q 退出，自动生成标定曲线

输出文件：
- distance_calibration.csv      # 原始标定数据
- distance_calibration_plot.png # 可视化曲线
"""

import cv2
import numpy as np
import sys
import time
import csv
from collections import deque

sys.path.insert(0, '/home/pi/RaspberryPi-CM5')

try:
    from picamera2 import Picamera2
    PICAM_AVAILABLE = True
except ImportError:
    PICAM_AVAILABLE = False
    print("[警告] Picamera2 不可用，尝试使用 OpenCV VideoCapture")

# ============================================================================
# 配置
# ============================================================================
FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2

# 红色 HSV 阈值（只识别红球）
COLOR_RANGES = [
    (np.array([0, 100, 60]),   np.array([10, 255, 255])),    # 红色低段
    (np.array([160, 100, 60]), np.array([180, 255, 255])),   # 红色高段
]

# ============================================================================
# 球检测器（优化版）
# ============================================================================
class BallDetector:
    def __init__(self):
        self.position_history = deque(maxlen=5)  # 防抖
    
    def detect(self, frame):
        """
        检测红球，返回 (cx, cy, area, circularity) 或 None
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 生成红色掩码
        mask = np.zeros((FRAME_H, FRAME_W), dtype=np.uint8)
        for lower, upper in COLOR_RANGES:
            mask |= cv2.inRange(hsv, lower, upper)
        
        # 形态学操作去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.dilate(mask, None, iterations=1)
        
        # 轮廓检测
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_ball = None
        best_score = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 250 or area > 25000:  # 面积过滤
                continue
            
            peri = cv2.arcLength(contour, True)
            if peri == 0:
                continue
            
            # 圆形度：(4πA)/(P²)，越接近1越圆
            circularity = (4 * np.pi * area) / (peri * peri)
            if circularity < 0.65:  # 严格要求
                continue
            
            x, y, w, h = cv2.boundingRect(contour)
            ratio = min(w, h) / max(w, h) if max(w, h) > 0 else 0
            if ratio < 0.55:  # 宽高比过滤
                continue
            
            # 获取圆心
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue
            
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            # 验证圆心在掩码内
            if mask[cy, cx] == 0:
                continue
            
            # 综合评分
            score = circularity * 20 + min(area, 5000) * 0.003
            
            if score > best_score:
                best_ball = (cx, cy, int(area), circularity)
                best_score = score
        
        return best_ball
    
    def smooth(self, ball_data):
        """防抖：使用历史数据平滑"""
        if ball_data is None:
            self.position_history.clear()
            return None
        
        cx, cy, area, circ = ball_data
        self.position_history.append((cx, cy))
        
        if len(self.position_history) < 3:
            return ball_data
        
        # 中位数平滑
        positions = list(self.position_history)
        cx_smooth = int(np.median([p[0] for p in positions]))
        cy_smooth = int(np.median([p[1] for p in positions]))
        
        return (cx_smooth, cy_smooth, area, circ)

# ============================================================================
# 距离标定主程序
# ============================================================================
def calibrate_distance():
    """
    交互式距离标定
    """
    # 初始化摄像头
    if PICAM_AVAILABLE:
        try:
            picam2 = Picamera2()
            picam2.preview_configuration.main.size = (FRAME_W, FRAME_H)
            picam2.preview_configuration.main.format = 'RGB888'
            picam2.configure('preview')
            picam2.start()
            time.sleep(1)
            print("[OK] Picamera2 就绪")
            
            def get_frame():
                rgb = picam2.capture_array()
                return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        except Exception as e:
            print(f"[错误] Picamera2 初始化失败: {e}，尝试 cv2.VideoCapture")
            cap = cv2.VideoCapture(0)
            cap.set(3, FRAME_W)
            cap.set(4, FRAME_H)
            def get_frame():
                ret, frame = cap.read()
                return frame if ret else None
    else:
        cap = cv2.VideoCapture(0)
        cap.set(3, FRAME_W)
        cap.set(4, FRAME_H)
        def get_frame():
            ret, frame = cap.read()
            return frame if ret else None
    
    # 初始化检测器
    detector = BallDetector()
    calibration_data = []  # 储存 (distance_cm, cy, area, circularity)
    
    print("\n" + "="*60)
    print("距离标定程序")
    print("="*60)
    print("步骤：")
    print("1. 把球放在狗前面，按空格键记录当前 cy 值")
    print("2. 输入距离（厘米）")
    print("3. 逐次移动球，重复步骤1-2")
    print("4. 按 q 退出，自动生成标定曲线")
    print("="*60 + "\n")
    
    frame_count = 0
    
    while True:
        frame = get_frame()
        if frame is None:
            print("[错误] 无法读取摄像头")
            break
        
        # 检测球
        ball_raw = detector.detect(frame)
        ball = detector.smooth(ball_raw)
        
        # 绘制界面
        display = frame.copy()
        
        # 画中心十字
        cv2.line(display, (CENTER_X, 0), (CENTER_X, FRAME_H), (180, 180, 180), 1)
        cv2.line(display, (0, FRAME_H//2), (FRAME_W, FRAME_H//2), (180, 180, 180), 1)
        
        # 画检测到的球
        if ball:
            cx, cy, area, circ = ball
            cv2.circle(display, (cx, cy), 20, (0, 255, 0), 2)
            cv2.drawMarker(display, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 30, 2)
            
            # 显示信息
            info_text = f"cx={cx:3d} cy={cy:3d} area={area:5d} circ={circ:.2f}"
            cv2.putText(display, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # 大字体显示 cy（关键参数）
            cy_text = f"cy = {cy}"
            cv2.putText(display, cy_text, (10, 80),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 200, 255), 3)
        else:
            cv2.putText(display, "NO BALL DETECTED", (10, 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        
        # 显示已标定的数据
        cv2.putText(display, f"Records: {len(calibration_data)}", (10, FRAME_H - 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.putText(display, "SPACE=record  q=quit", (10, FRAME_H - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        
        cv2.imshow('Distance Calibration', display)
        
        # 键盘事件
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            print("\n[用户] 退出标定")
            break
        
        elif key == 32:  # 空格键
            if ball:
                cx, cy, area, circ = ball
                # 询问用户输入距离
                print(f"\n检测到球: cx={cx}, cy={cy}, area={area}, circ={circ:.2f}")
                try:
                    distance_cm = int(input("输入距离（厘米，例如 30）: "))
                    if distance_cm > 0 and distance_cm < 200:
                        calibration_data.append({
                            'distance_cm': distance_cm,
                            'cy': cy,
                            'area': area,
                            'circularity': circ
                        })
                        print(f"✓ 已记录: {distance_cm}cm → cy={cy}")
                    else:
                        print("✗ 距离无效")
                except ValueError:
                    print("✗ 请输入数字")
            else:
                print("✗ 未检测到球")
        
        frame_count += 1
    
    cv2.destroyAllWindows()
    
    # 保存标定数据
    if calibration_data:
        print("\n" + "="*60)
        print("标定数据汇总")
        print("="*60)
        print(f"{'距离(cm)':<12} {'cy值':<12} {'面积':<12} {'圆形度':<12}")
        print("-"*60)
        
        for record in calibration_data:
            print(f"{record['distance_cm']:<12} {record['cy']:<12} {record['area']:<12} {record['circularity']:<12.2f}")
        
        # 保存为 CSV
        csv_path = '/home/pi/distance_calibration.csv'
        try:
            with open(csv_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['distance_cm', 'cy', 'area', 'circularity'])
                writer.writeheader()
                writer.writerows(calibration_data)
            print(f"\n✓ 数据已保存到 {csv_path}")
        except Exception as e:
            print(f"\n✗ 保存失败: {e}")
        
        # 生成标定曲线
        try:
            import matplotlib.pyplot as plt
            
            distances = [r['distance_cm'] for r in calibration_data]
            cy_values = [r['cy'] for r in calibration_data]
            
            # 拟合多项式
            coeffs = np.polyfit(cy_values, distances, 2)  # 二次多项式
            poly = np.poly1d(coeffs)
            
            print(f"\n标定结果（二次多项式）:")
            print(f"distance = {coeffs[0]:.6f}*cy² + {coeffs[1]:.6f}*cy + {coeffs[2]:.6f}")
            
            # 绘制
            fig, ax = plt.subplots(figsize=(10, 6))
            
            # 原始数据点
            ax.scatter(cy_values, distances, color='red', s=100, label='实测数据', zorder=5)
            
            # 拟合曲线
            cy_range = np.linspace(min(cy_values) - 20, max(cy_values) + 20, 200)
            distance_fit = poly(cy_range)
            ax.plot(cy_range, distance_fit, 'b-', linewidth=2, label='二次拟合曲线')
            
            # 标签
            ax.set_xlabel('cy 值（像素）', fontsize=12)
            ax.set_ylabel('距离（厘米）', fontsize=12)
            ax.set_title('球距离标定曲线', fontsize=14, fontweight='bold')
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=11)
            
            # 标注数据点
            for i, (cy, dist) in enumerate(zip(cy_values, distances)):
                ax.annotate(f'{dist}cm', (cy, dist), textcoords="offset points",
                           xytext=(0, 10), ha='center', fontsize=9)
            
            plot_path = '/home/pi/distance_calibration_plot.png'
            fig.savefig(plot_path, dpi=150, bbox_inches='tight')
            print(f"✓ 曲线图已保存到 {plot_path}")
            plt.close()
            
            # 保存标定系数
            coeff_path = '/home/pi/calibration_coeffs.txt'
            with open(coeff_path, 'w') as f:
                f.write(f"# 距离标定系数（二次多项式）\n")
                f.write(f"# distance = a*cy² + b*cy + c\n")
                f.write(f"a = {coeffs[0]:.10f}\n")
                f.write(f"b = {coeffs[1]:.10f}\n")
                f.write(f"c = {coeffs[2]:.10f}\n")
            print(f"✓ 标定系数已保存到 {coeff_path}")
            
        except ImportError:
            print("\n[提示] matplotlib 未安装，跳过曲线绘制")
        except Exception as e:
            print(f"\n✗ 曲线生成失败: {e}")
        
        print("\n" + "="*60)
        print("标定完成！")
        print("="*60)
    else:
        print("\n[警告] 未记录任何标定数据")

if __name__ == '__main__':
    try:
        calibrate_distance()
    except KeyboardInterrupt:
        print("\n[用户中断]")
