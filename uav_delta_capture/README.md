# uav_delta_capture

基于 ROS 2 Humble 的"无人机挂载 Delta 机械臂协同抓取"项目，面向 STM32MP257F-DK（Cortex-A35 + NPU）平台。

系统流水线：**视觉感知 → 坐标变换 → 安全门控 → 逆运动学 → 关节角度**

## 工作空间结构

```text
uav_delta_capture/
└── src/
    ├── uav_delta_msgs/        # 自定义消息与服务（GraspTarget.msg, SetArmStatus.srv）
    ├── delta_kinematics/      # Delta 并联机构正/逆运动学（C++ / Eigen）
    ├── perception_logic/      # 视觉感知原型（Python，预留 NPU 推理接口）
    ├── uav_bridge/            # UAV 状态桥接：TF2 变换 / FCU 健康监控 / 飞行状态机 / 安全管理器
    └── vision_bench/          # 视觉推理性能测试（Python / ONNX Runtime，NPU 自动检测）
```

## 包说明

| 包名 | 语言 | 功能 |
|------|------|------|
| `uav_delta_msgs` | C++ (rosidl) | 自定义消息 `GraspTarget` 和服务 `SetArmStatus` |
| `delta_kinematics` | C++ | 3-DOF Delta 并联机构正/逆运动学求解，50Hz 定时器驱动 |
| `perception_logic` | Python | 视觉感知节点，订阅相机图像，发布目标像素偏移（NPU 推理桩函数） |
| `uav_bridge` | C++ | 包含 5 个独立节点：坐标变换桥接、FCU 链路监控、飞行状态机、安全管理器、Mock 飞控位姿 |
| `vision_bench` | Python | ONNX Runtime 推理性能测试，支持 YOLOv8n，NPU 自动检测，系统资源监控 |

## 节点架构

```text
┌──────────────┐     ┌──────────────────┐     ┌───────────────────┐
│  相机图像     │────▶│  perception_node │────▶│ vision/target_    │
│ /camera/     │     │  (Python)        │     │   offset          │
│ image_raw    │     └──────────────────┘     └────────┬──────────┘
└──────────────┘                                      │
                                                      ▼
┌──────────────┐     ┌──────────────────┐     ┌───────────────────┐
│ MAVROS 位姿  │────▶│  uav_bridge_node │────▶│   target_point    │
│ MAVROS IMU   │     │  (C++ / tf2)     │     │   local_attitude  │
└──────────────┘     └──────────────────┘     └────────┬──────────┘
                                                       │
┌──────────────┐     ┌──────────────────┐              ▼
│ Mock 飞控    │────▶│ mock_mavros_pose │     ┌───────────────────┐
│ (测试用)     │     │ (C++)            │     │ failsafe_manager  │
└──────────────┘     └──────────────────┘     │ (C++)             │
                                               │ ─ target_point_safe
┌──────────────┐     ┌──────────────────┐     └────────┬──────────┘
│ FCU 链路状态  │────▶│ flight_state_    │              │
│              │     │ machine (C++)    │              ▼
└──────────────┘     └──────────────────┘     ┌───────────────────┐
                                               │ delta_kinematics  │
┌──────────────┐     ┌──────────────────┐     │ (C++ / Eigen)     │
│ FCU 心跳监控  │────▶│ fcu_link_monitor │     │ ─ joint_angles    │
│              │     │ (C++)            │     └───────────────────┘
└──────────────┘     └──────────────────┘
```

## Topic 列表

| Topic | 类型 | 发布者 | 说明 |
|-------|------|--------|------|
| `/mavros/local_position/pose` | PoseStamped | MAVROS / mock_mavros_pose | 飞控本地位姿 |
| `vision/target_offset` | PointStamped | perception_node | 视觉目标像素偏移 |
| `target_point` | PointStamped | uav_bridge_node | 变换后的目标点（delta_base_link） |
| `target_point_safe` | PointStamped | failsafe_manager | 安全门控后的目标点 |
| `joint_angles` | Float64MultiArray | delta_kinematics_node | 3 个关节角度（度） |
| `local_attitude` | Vector3Stamped | uav_bridge_node | 欧拉角姿态（roll/pitch/yaw） |
| `imu_attitude` | Vector3Stamped | uav_bridge_node | IMU 姿态 |
| `fcu_link/status` | String | fcu_link_monitor | WAIT_FCU / OK / LOST |
| `uav_bridge/flight_state` | String | flight_state_machine | INIT / WAIT_FCU / TRACKING / FAILSAFE |

## 自定义消息与服务

**GraspTarget.msg**
```text
geometry_msgs/Point position
int32 target_id
```

**SetArmStatus.srv**
```text
int32 IDLE=0
int32 GRASP=1
int32 RESET=2
int32 status
---
bool success
string message
```

## 构建与运行

### Docker 容器内构建

```bash
docker exec -it ros2humble bash -c "
  cd /workspace/uav_delta_capture &&
  source /opt/ros/humble/setup.bash &&
  colcon build --symlink-install --parallel-workers 1
"
```

### 运行

```bash
docker exec -it ros2humble bash -c "
  source /opt/ros/humble/setup.bash &&
  source install/setup.bash &&
  ros2 launch uav_bridge uav_delta_system.launch.py use_mock_fcu:=true start_mavros:=false
"
```

### 启动模式

**Mock 飞控（开发/测试，无数传）：**
```bash
ros2 launch uav_bridge uav_delta_with_mock_fcu.launch.py
```

**真实飞控（需已安装 MAVROS）：**
```bash
ros2 launch uav_bridge uav_delta_with_fcu.launch.py
```

**自定义参数启动：**
```bash
ros2 launch uav_bridge uav_delta_system.launch.py \
  use_mock_fcu:=true \
  start_mavros:=false \
  start_fcu_guard:=true
```

### vision_bench 性能测试

```bash
docker exec -it ros2humble bash -c "
  cd /workspace/uav_delta_capture &&
  source /opt/ros/humble/setup.bash &&
  colcon build --packages-select vision_bench --symlink-install --parallel-workers 1 &&
  source install/setup.bash &&
  pip install onnxruntime &&
  ros2 launch vision_bench vision_bench.launch.py
"
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mode` | `synthetic` | `synthetic` 合成图像 / `subscribe` 订阅真实相机 |
| `input_size` | `320` | 推理输入分辨率 |
| `use_npu` | `false` | 尝试 NPU 加速（需 X-LINUX-AI） |
| `num_iterations` | `100` | 推理轮数 |
| `model_path` | `""` | 空则自动下载 YOLOv8n |
| `conf_thresh` | `0.25` | 检测置信度阈值 |

## 关键配置文件

| 文件 | 节点 | 主要参数 |
|------|------|----------|
| [delta_kinematics.yaml](src/delta_kinematics/config/delta_kinematics.yaml) | delta_kinematics_node | `L1=100`, `L2=150`, `R=55`, `r=20`（mm） |
| [perception_logic.yaml](src/perception_logic/config/perception_logic.yaml) | perception_node | `image_topic`, `publish_topic`, `camera_frame` |
| [uav_bridge.yaml](src/uav_bridge/config/uav_bridge.yaml) | uav_bridge_node | `serial_port`, `baudrate`, `camera_frame`, `target_frame` |
| [mock_mavros_pose.yaml](src/uav_bridge/config/mock_mavros_pose.yaml) | mock_mavros_pose_node | `trajectory_mode`, `rate`, `origin_*`, `amplitude_*` |
| [fcu_health.yaml](src/uav_bridge/config/fcu_health.yaml) | fcu_link_monitor / flight_state_machine | `timeout_sec`, `threshold_count`, `publish_rate_hz` |
| [failsafe.yaml](src/uav_bridge/config/failsafe.yaml) | failsafe_manager_node | `input_timeout_sec=0.4`, `safe_x/y/z` |
| [mavros_bridge.yaml](src/uav_bridge/config/mavros_bridge.yaml) | mavros_node | `fcu_url`, `system_id`, `component_id` |
| [vision_bench.yaml](src/vision_bench/config/vision_bench.yaml) | vision_bench_node | `input_size`, `use_npu`, `num_iterations` |

## 联调检查

```bash
ros2 topic hz /mavros/local_position/pose
ros2 topic echo fcu_link/status
ros2 topic echo uav_bridge/flight_state
ros2 topic echo target_point
ros2 topic echo target_point_safe
ros2 topic echo joint_angles
```

## 模型文件

`models/` 目录包含预置 ONNX 模型：

| 文件 | 大小 | 说明 |
|------|------|------|
| `yolov8n.onnx` | 12.85 MB | YOLOv8n 预训练模型（COCO 80 类） |
| `yolov8n_320.onnx` | 12.85 MB | YOLOv8n（320px 输入） |
| `yolov8n_synthetic.onnx` | 8.47 MB | 轻量回退模型 |

vision_bench 模型加载优先级：用户指定路径 → 自动下载到 `~/.cache/vision_bench/models/` → 回退到 `models/yolov8n_synthetic.onnx`

## 依赖

| 依赖 | 用途 |
|------|------|
| `rclcpp` / `rclpy` | ROS 2 客户端库 |
| `geometry_msgs` / `sensor_msgs` / `std_msgs` | 标准消息类型 |
| `tf2` / `tf2_ros` / `tf2_geometry_msgs` | 坐标变换 |
| `mavros` | 飞控通信（仅 uav_bridge） |
| `eigen3` | 矩阵运算（仅 delta_kinematics） |
| `onnxruntime` | 视觉推理（仅 vision_bench） |
| `rosidl_default_generators` | 消息/服务代码生成（仅 uav_delta_msgs） |

## 编码约束

- C++ 节点使用 `rclcpp::Timer` 驱动，无 `while` 主循环
- 高频回调避免大内存分配，复用消息缓存
- 串口路径与波特率参数化，未硬编码
- 使用 `RCLCPP_INFO` / `RCLCPP_DEBUG` 结构化日志
- `--parallel-workers 1` 构建，适配板卡有限内存
