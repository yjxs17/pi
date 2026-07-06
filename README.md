# XGO-Lite 四足机器人控制系统

## 项目概述

本项目为 2026年国赛智能机器人创意大赛（四足小型组）的 XGO-Lite 机器人软件解决方案。

### 核心任务流程

1. **搜索小球** — 球不在画面时原地转圈
2. **站立靠近** — 正常姿势走过去，球逐渐进入视野
3. **蹲下前趴** — 降低重心，获得更大的末端工作空间
4. **蹲下微调** — 精细定位，确保球在抓取位置
5. **机械臂抓取** — 视觉盲抓，渐进闭合防弹飞
6. **防掉落搬运** — 5层防护措施确保物体安全
7. **精准放置** — 视觉引导定位，放入目标区域

### 文件结构

```
pi/
├── README.md                          # 本文件
├── arm_control.py                     # 机械臂控制核心模块
├── auto_grab.py                       # 自动抓球完整流程
├── ball_debug_tune.py                 # 球颜色阈值调试工具
├── debug_cy.py                        # cy值实时监控工具
├── debug_cy_overlay.py                # cy值画面叠加显示
├── debug_cy_web.py                    # cy值网页测量工具
├── cam_server.py                      # 摄像头网页推流服务
├── peek_*.py                          # 蹲下姿态保持脚本
├── test*.py                           # 测试脚本集合
├── xgolib.py                          # XGO-Lite 模拟库（PC调试用）
├── 001.py                             # 舞蹈演示程序
└── 机械臂控制模块技术方案.html         # 详细技术文档
```

## 核心模块

### arm_control.py - 机械臂控制

**主要功能：**
- `safe_init(dog)` — 安全初始化
- `safe_shutdown(dog)` — 安全关机
- `catch_arm(dog, obj_type)` — 抓取流程
- `start_transport(dog)` — 搬运启动
- `down_arm(dog)` — 放置流程
- `arm_main_loop()` — 完整主流程

**5层防掉落机制：**
1. 夹爪力度定期维持
2. IMU 动态平衡
3. 低重心行走
4. 慢步态行走
5. 转弯姿态补偿

### auto_grab.py - 完整自动抓球

集成视觉检测 + 运动控制的端到端方案。

**VisionModule：** 颜色检测 + Canny边缘 + 圆形度评分

**MotionModule：** 舞蹈控制库的高级封装

### 调试工具

| 工具 | 功能 | 用法 |
|------|------|------|
| `ball_debug_tune.py` | HSV阈值实时调试 | `python3 ball_debug_tune.py red` |
| `debug_cy.py` | cy值终端输出 | `python3 debug_cy.py` |
| `debug_cy_overlay.py` | cy值画面显示 | `python3 debug_cy_overlay.py` |
| `debug_cy_web.py` | cy值网页测量 | `python3 debug_cy_web.py` → http://IP:8090 |
| `cam_server.py` | 实时摄像头推流 | `python3 cam_server.py` → http://IP:8080 |

## 参数标定

阶段一必须完成 7 项标定：

1. **arm_polar 坐标系方向** — 确认 theta 起算
2. **参数范围验证** — theta:70~270, r:80~140
3. **claw ��度标定** — 小球(180) vs 长条(235)
4. **舵机响应延迟** — 确认 safe_claw 间隔
5. **死区宽度** — deadzone 补偿参数
6. **电量返回值映射** — 充满 vs 低电量
7. **RED_AREA_THRESHOLD** — 视觉确认阈值

## 运行指南

### 树莓派真机运行

```bash
# 基础抓取测试
sudo python3 -c "from arm_control import *; dog = XGO('/dev/ttyAMA0'); safe_init(dog); catch_arm(dog); safe_shutdown(dog)"

# 完整流程（需运动模块配合）
sudo python3 arm_control.py --zone=A

# 自动抓球
sudo python3 auto_grab.py
```

### Windows/PC 模拟运行

```bash
# 直接运行（自动进入模拟模式）
python arm_control.py --sim --test-grasp
python arm_control.py --sim --test-arm
```

### 摄像头实时监控

```bash
# 启动网页推流
python3 cam_server.py

# 浏览器打开
http://192.168.X.X:8080
```

## 关键技术约束

| 约束 | 说明 | 影响 |
|------|------|------|
| 摄像头单占用 | 同一时间只能被一个程序占用 | 视觉检测必须在同一进程 |
| turn(-8) 不可用 | 左转API故障 | 用 turn(8) + 前进组合替代 |
| 蹲下后视觉盲区 | 摄像头无法看到地面 | 最后阶段必须闭眼盲抓 |
| 中文文件名限制 | scp 传输失败 | 使用英文文件名 |
| 当前IP | 192.168.210.213 | 网页访问地址 |

## 技术文档

详细的技术方案请参考 `机械臂控制模块技术方案.html`（在浏览器打开），包括：

- API 参考
- 函数文档（20+个函数详解）
- 调用链关系图
- 舵机扭矩分析
- 风险评估与应对
- 8周分阶段实施计划
- 标定任务清单
- 迭代记录 (v1.0 → v2.0)

## 实现进度

✅ **已完成：**
- 机械臂基础控制库
- 安全初始化/关机
- 抓取/放置流程
- 视觉确认机制
- 通信保护包装
- 电量监控自适应
- 异常恢复流程
- 测试工具集

⏳ **进行中：**
- 运动模块对接
- 搬运防掉落验证
- 区域标记识别
- 放置精度优化

❌ **待做：**
- 完整端到端集成
- 比赛场景适配
- 连续运行稳定性测试

## 故障排查

### 问题：机械臂无响应

**原因：** 舵机带电/已掉电/串口连接不稳定

**解决：** 
1. 检查 `/dev/ttyAMA0` 权限：`ls -l /dev/ttyAMA0`
2. 执行 `safe_init(dog)` 重新初始化
3. 检查硬件连接

### 问题：抓取失败率高

**原因：** HSV 阈值不当、claw 力度不足、死区设置错误

**解决：**
1. 运行 `ball_debug_tune.py` 优化 HSV
2. 调高 claw 值（max 245）
3. 增大 deadzone 宽度
4. 运行 `test_static_grasp()` 验证参数

### 问题：搬运中掉球

**原因：** 缺少转弯补偿、夹持力下降、低电量

**解决：**
1. 确保 `on_turning()` 被调用
2. 每 2.5s 调用 `transport_tick()` 维持夹力
3. 检查电量，低电量自动增大夹持力

## 参考资源

- XGO-Lite 官方文档
- 树莓派 CM4 系统配置
- OpenCV 图像处理指南
- Flask 网页框架

## 许可证

MIT License

## 联系方式

开发者：冰零❄️ + 冰灵🔧

---

**最后更新：** 2026-07-06

**版本：** 2.0 (arm_control 集成版)
