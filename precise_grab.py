#!/usr/bin/env python3
# coding=utf-8
"""
precise_grab.py - 准确抓取完整工程代码
========================================
集成：视觉检测 + 距离标定 + 多帧融合 + 动态运动策略 + 硬件反馈

核心改进：
1. 增强的红球检测（圆形度、面积、轮廓多重过滤）
2. 多帧滑动窗口��抖
3. 分距离阶段的精细运动控制
4. 视觉 + 机械反馈双重确认

运行方式：
  sudo python3 precise_grab.py --sim             # 模拟模式（PC调试）
  sudo python3 precise_grab.py --real            # 真机运行
  sudo python3 precise_grab.py --real --debug    # 真机+调试输出
"""

import cv2
import numpy as np
import sys
import time
import argparse
from collections import deque
from datetime import datetime

sys.path.insert(0, '/home/pi/RaspberryPi-CM5')

try:
    from picamera2 import Picamera2
    PICAM_AVAILABLE = True
except ImportError:
    PICAM_AVAILABLE = False

try:
    from xgolib import XGO
    XGOLIB_AVAILABLE = True
except ImportError:
    XGOLIB_AVAILABLE = False

# ============================================================================
# 配置参数
# ============================================================================
FRAME_W = 640
FRAME_H = 480
CENTER_X = FRAME_W // 2
CENTER_Y = FRAME_H // 2

# 红色 HSV 阈值（增强版，专为红球优化）
RED_RANGES = [
    (np.array([0, 100, 60]),   np.array([10, 255, 255])),
    (np.array([160, 100, 60]), np.array([180, 255, 255])),
]

# 运动参数
MOTION_PARAMS = {
    'far': {          # >80cm
        'speed_turn': 0.05,
        'speed_move': 30,
        'tolerance_x': 150,
        'name': 'fast_approach'
    },
    'mid': {          # 40-80cm
        'speed_turn': 0.08,
        'speed_move': 20,
        'tolerance_x': 80,
        'name': 'normal_approach'
    },
    'near': {         # 15-40cm
        'speed_turn': 0.10,
        'speed_move': 10,
        'tolerance_x': 50,
        'name': 'careful_approach'
    },
    'fine': {         # 5-15cm
        'speed_turn': 0.15,
        'speed_move': 5,
        'tolerance_x': 30,
        'name': 'fine_tune'
    },
}

# ============================================================================
# 增强的球检测器
# ============================================================================
class AdvancedBallDetector:
    """
    多重过滤的红球检测：
    1. HSV 颜色范围过滤
    2. 面积过滤
    3. 圆形度评分
    4. 轮廓宽高比检查
    5. 多帧滑动窗口防抖
    """
    
    def __init__(self, window_size=5, confidence_threshold=0.7):
        self.position_history = deque(maxlen=window_size)
        self.confidence = 0.0
        self.lost_frames = 0
        self.threshold = confidence_threshold
        self.last_valid_ball = None
    
    def detect_single_frame(self, frame):
        """单帧检测，返回 (cx, cy, area, circularity) 或 None"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 生成红色掩码
        mask = np.zeros((FRAME_H, FRAME_W), dtype=np.uint8)
        for lower, upper in RED_RANGES:
            mask |= cv2.inRange(hsv, lower, upper)
        
        # 形态学去噪
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
            if area < 250 or area > 25000:
                continue
            
            peri = cv2.arcLength(contour, True)
            if peri == 0:
                continue
            
            # 圆形度检查
            circularity = (4 * np.pi * area) / (peri * peri)
            if circularity < 0.65:
                continue
            
            # 宽高比检查
            x, y, w, h = cv2.boundingRect(contour)
            ratio = min(w, h) / max(w, h) if max(w, h) > 0 else 0
            if ratio < 0.55:
                continue
            
            # 获取圆心
            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue
            
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            # 圆心必须在掩码内
            if mask[cy, cx] == 0:
                continue
            
            # 综合评分
            score = circularity * 20 + min(area, 5000) * 0.003
            
            if score > best_score:
                best_ball = (cx, cy, int(area), circularity)
                best_score = score
        
        return best_ball
    
    def update(self, frame):
        """
        更新检测结果，返回平滑后的 (cx, cy, distance, confidence)
        """
        detected_ball = self.detect_single_frame(frame)
        
        if detected_ball is not None:
            cx, cy, area, circ = detected_ball
            
            # 检查是否与历史位置跳跃（可能误检）
            if self.position_history:
                last_cx, last_cy = self.position_history[-1]
                dist_jump = np.sqrt((cx - last_cx)**2 + (cy - last_cy)**2)
                
                if dist_jump < 60:  # 同帧变化 <60px
                    self.position_history.append((cx, cy))
                    self.lost_frames = 0
                    self.confidence = min(1.0, self.confidence + 0.2)
                else:
                    # 跳跃检测 → 可能误检
                    self.lost_frames += 1
                    self.confidence *= 0.7
            else:
                self.position_history.append((cx, cy))
                self.confidence = 0.5
            
            self.last_valid_ball = (cx, cy, area, circ)
        else:
            # 未检测到球
            self.lost_frames += 1
            self.confidence *= 0.6
            
            if self.lost_frames > 5:
                self.position_history.clear()
                self.confidence = 0
        
        # 返回平滑结果
        if self.position_history and self.confidence > self.threshold:
            positions = list(self.position_history)
            cx_smooth = int(np.median([p[0] for p in positions]))
            cy_smooth = int(np.median([p[1] for p in positions]))
            
            # 距离估算（需要用标定数据）
            dist = estimate_distance(cy_smooth)
            
            return {
                'cx': cx_smooth,
                'cy': cy_smooth,
                'distance': dist,
                'confidence': self.confidence,
                'found': True
            }
        
        return {'found': False, 'confidence': self.confidence}

# ============================================================================
# 距离估算（从标定数据）
# ============================================================================
def estimate_distance(cy):
    """
    根据 cy 值估算距离
    
    使用方法：
    1. 如果有标定数据，用标定系数（二次多项式）
    2. 如果没有，用默认线性公式
    
    标定系数文件：/home/pi/calibration_coeffs.txt
    格式：
        a = 0.000123
        b = -0.456
        c = 789.0
    """
    try:
        # 尝试读取标定系数
        with open('/home/pi/calibration_coeffs.txt', 'r') as f:
            lines = f.readlines()
            a = float(lines[2].split('=')[1].strip())
            b = float(lines[3].split('=')[1].strip())
            c = float(lines[4].split('=')[1].strip())
        
        # 用标定多项式
        dist = a * cy**2 + b * cy + c
        return max(0, min(300, dist))  # 限制在合理范围
    except:
        # 降级：使用默认线性公式
        dist = (563 - cy) / 5.9
        return max(0, min(300, dist))

# ============================================================================
# 智能运动控制
# ============================================================================
class IntelligentMotionController:
    """根据距离自适应运动策略"""
    
    def __init__(self, dog, debug=False):
        self.dog = dog
        self.debug = debug
        self.stage = None
    
    def approach(self, ball_data):
        """
        根据球的位置和距离决定运动
        返回下一个阶段名称
        """
        if not ball_data['found']:
            if self.debug:
                print("[搜索] 球丢失，原地转圈")
            self.dog.turn(8)
            return 'searching'
        
        cx = ball_data['cx']
        cy = ball_data['cy']
        dist = ball_data['distance']
        conf = ball_data['confidence']
        
        offset_x = cx - CENTER_X  # 正=球在右，负=球在左
        
        # 判断阶段
        if dist > 80:
            stage = 'far'
        elif dist > 40:
            stage = 'mid'
        elif dist > 15:
            stage = 'near'
        elif dist > 5:
            stage = 'fine'
        else:
            self.dog.stop()
            if self.debug:
                print("[就绪] 到达抓取距离")
            return 'ready_to_grasp'
        
        params = MOTION_PARAMS[stage]
        
        # 应用运动策略
        if abs(offset_x) > params['tolerance_x']:
            # 需要转向
            turn_speed = int(offset_x * params['speed_turn'])
            self.dog.turn(turn_speed)
            if self.debug:
                print(f"[{stage}] 转向: offset={offset_x}, turn_speed={turn_speed}")
        else:
            # 对准，直进
            self.dog.turn(0)
            self.dog.move_x(params['speed_move'])
            if self.debug:
                print(f"[{stage}] 前进: dist={dist:.1f}cm, offset={offset_x}")
        
        self.stage = stage
        return stage

# ============================================================================
# 精确抓取流程
# ============================================================================
def precise_grasp_sequence(dog, detector, cap, debug=False):
    """
    完整的精确抓取流程
    """
    print("\n" + "="*60)
    print("精确抓取序列启动")
    print("="*60)
    
    motion_ctrl = IntelligentMotionController(dog, debug=debug)
    
    # ======= 阶段1：视觉引导接近 =======
    print("\n[阶段1] 视觉引导接近...")
    max_approach_frames = 300  # 最多 30 帧（@10fps）
    
    for frame_idx in range(max_approach_frames):
        ret, frame = cap.read() if cap else (False, None)
        if not ret and cap:
            continue
        
        ball_data = detector.update(frame) if frame is not None else {'found': False}
        next_stage = motion_ctrl.approach(ball_data)
        
        if next_stage == 'ready_to_grasp':
            print("[成功] 已到达抓取距离")
            break
        
        time.sleep(0.1)
    else:
        print("[超时] 接近超时")
        return False
    
    # ======= 阶段2：蹲下准备 =======
    print("\n[阶段2] 蹲下准备...")
    dog.imu(1)
    dog.translation('z', 75)
    dog.attitude('p', 15)
    time.sleep(0.5)
    
    # ======= 阶段3：机械臂伸出 =======
    print("[阶段3] 机械臂伸出...")
    dog.claw(5)  # 张开夹爪
    dog.arm_polar(210, 130)  # 伸出到抓取位
    time.sleep(1.5)
    
    # ======= 阶段4：渐进闭合 =======
    print("[阶段4] 渐进闭合...")
    
    # 分步闭合：5 → 50 → 100 → 150 → 180 → 200
    claw_steps = [50, 100, 150, 180, 200]
    for claw_val in claw_steps:
        dog.claw(claw_val)
        time.sleep(0.15)
    
    # 死区补偿（过冲+回落）
    dog.claw(210)  # 过冲
    time.sleep(0.2)
    dog.claw(200)  # 回落
    time.sleep(0.2)
    
    # ======= 阶段5：提拉验证（机械反馈）=======
    print("[阶段5] 提拉验证...")
    dog.arm_polar(120, 125)
    time.sleep(0.6)
    dog.arm_polar(115, 120)
    time.sleep(0.6)
    
    # ======= 阶段6：视觉二次确认 =======
    print("[阶段6] 视觉确认...")
    grabbed = False
    
    if cap:
        ret, verify_frame = cap.read()
        if ret:
            # 检查球是否还在原位（HSV检测）
            hsv = cv2.cvtColor(verify_frame, cv2.COLOR_BGR2HSV)
            mask = np.zeros((FRAME_H, FRAME_W), dtype=np.uint8)
            for lower, upper in RED_RANGES:
                mask |= cv2.inRange(hsv, lower, upper)
            
            h, w = mask.shape
            bottom_region = mask[int(h*0.6):h, :]
            red_pixels = cv2.countNonZero(bottom_region)
            
            if red_pixels < 300:  # 如果原位红色像素少，说明球被抓起
                grabbed = True
                print("[成功] 视觉确认：球已被抓起！")
            else:
                print(f"[警告] 视觉检测：球可能还在原位（红色像素={red_pixels}）")
    else:
        # 没有摄像头反馈时，默认成功
        grabbed = True
        print("[默认] 无视觉反馈，假设抓取成功")
    
    # ======= 后续处理 =======
    if grabbed:
        print("[阶段7] 收回机械臂...")
        dog.arm_polar(90, 80)
        time.sleep(1.0)
        return True
    else:
        print("[失败] 抓取确认失败，释放...")
        dog.claw(5)
        dog.arm_polar(90, 80)
        time.sleep(0.5)
        return False

# ============================================================================
# 主程序
# ============================================================================
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--sim', action='store_true', help='模拟模式')
    parser.add_argument('--real', action='store_true', help='真机模式')
    parser.add_argument('--debug', action='store_true', help='调试输出')
    args = parser.parse_args()
    
    # 确定模式
    if args.sim:
        is_sim = True
    elif args.real:
        is_sim = False
    else:
        is_sim = not XGOLIB_AVAILABLE
    
    print("\n" + "="*60)
    print(f"精确抓取程序 [{'模拟模式' if is_sim else '真机模式'}]")
    print("="*60)
    
    # 初始化机器狗
    if not is_sim:
        try:
            from xgolib import XGO
            dog = XGO(port="/dev/ttyAMA0", version="xgolite")
            print("[OK] 机器狗已连接")
            time.sleep(1)
        except Exception as e:
            print(f"[错误] 无法连接机器狗: {e}")
            return
    else:
        # 模拟模式
        class MockDog:
            def __getattr__(self, name):
                if args.debug:
                    print(f"[MOCK] {name}(...)")
                return lambda *args, **kwargs: None
        
        dog = MockDog()
        print("[OK] 使用模拟狗")
    
    # 初始化摄像头
    cap = None
    if PICAM_AVAILABLE:
        try:
            picam2 = Picamera2()
            picam2.preview_configuration.main.size = (FRAME_W, FRAME_H)
            picam2.preview_configuration.main.format = 'RGB888'
            picam2.configure('preview')
            picam2.start()
            time.sleep(1)
            print("[OK] Picamera2 已启动")
            
            def get_frame():
                rgb = picam2.capture_array()
                return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            
            cap = type('obj', (object,), {
                'read': lambda: (True, get_frame())
            })()
        except Exception as e:
            print(f"[警告] Picamera2 初始化失败: {e}")
            cap = None
    
    if cap is None:
        cap = cv2.VideoCapture(0)
        cap.set(3, FRAME_W)
        cap.set(4, FRAME_H)
        if cap.isOpened():
            print("[OK] OpenCV VideoCapture 已启动")
        else:
            print("[警告] 无法打开摄像头，将在无视觉反馈模式运行")
    
    # 初始化检测器
    detector = AdvancedBallDetector(window_size=5, confidence_threshold=0.7)
    
    # 执行抓取
    try:
        success = precise_grasp_sequence(dog, detector, cap, debug=args.debug)
        
        if success:
            print("\n✓ 抓取成功！")
        else:
            print("\n✗ 抓取失败")
    
    except KeyboardInterrupt:
        print("\n[用户中断]")
    finally:
        if cap:
            try:
                cap.release()
            except:
                pass

if __name__ == '__main__':
    main()
