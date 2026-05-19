<details open>
<summary><b>🇬🇧 English</b> &nbsp;|&nbsp; <a href="#chinese"><b>🇨🇳 中文</b></a></summary>

# BrickPick

<img width="400" height="225" alt="4613_1779098093-ezgif com-optimize" src="https://github.com/user-attachments/assets/d433e084-e983-4fe5-bb63-a5573a930ec6" />



[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=c%2B%2B)](https://isocpp.org/)
[![BehaviorTree.CPP](https://img.shields.io/badge/BT.CPP-4.x-green)](https://www.behaviortree.dev/)
[![YOLO](https://img.shields.io/badge/YOLO-11-orange?logo=ultralytics)](https://docs.ultralytics.com/)
[![License](https://img.shields.io/badge/License-Apache_2.0-lightgrey)](LICENSE)

Autonomous brick-picking system for a RoboMaster EP mobile manipulator — vision-guided navigation, object approach with blind-zone takeover, and arm sequencing orchestrated by a Behavior Tree.

---

## System Architecture

### Core ROS 2 Nodes

| Node | Language | Role |
|------|----------|------|
| `brickpick_vision` | Python | YOLO inference on camera frames; publishes `Detection2DArray` and annotated debug images |
| `find_node` | Python | In-place rotation + multi-frame debounce; confirms a valid detection before reporting SUCCESS |
| `approach_node` | Python | State machine (`IDLE → ALIGN → APPROACH → EXTEND → DONE`): yaw-alignment, forward drive, blind extension when the target enters the camera's dead zone |
| `arm_preset_node` | Python | Preset-sequence arm controller with reentrant callback groups; executes configurable position sequences via `MoveArm` action |
| `odom_2d_filter` | Python | Projects 6-DoF odometry to a 2D transform (`odom_2d → base_link`) by stripping Z, roll, and pitch |
| `teleop_keyboard_node` | Python | Incremental keyboard teleoperation (TurtleBot3-style) for manual testing |
| `image_capture_node` | Python | Keyboard-triggered frame capture for dataset collection |
| `brickpick_bt_executor` | C++ | Behavior Tree engine; loads XML trees, registers custom nodes, and drives the mission loop at 50 Hz |

### Behavior Tree Custom Nodes

| Node | Type | Description |
|------|------|-------------|
| `ReadRegionCenters` | Action | Parses `region_centers.txt` into a port-accessible coordinate vector |
| `HasNextCenterPoint` | Condition | Checks whether `current_index` is still within the regions vector |
| `NavToIndexedPoseClient` | Action | Sends a Nav2 `NavigateToPose` goal for the indexed region center |
| `FindObject` | Action | Invokes `find_node` via Trigger service; polls status topic |
| `ApproachObject` | Action | Invokes `approach_node` via Trigger service; polls status topic |
| `ExecuteArmSequence` | Action | Invokes `arm_preset_node` via Trigger service; polls status topic |
| `IncrementIndex` | Action | Bidirectional port: reads `current_index`, writes `current_index + 1` |
| `ReturnToStartPosition` | Action | Navigates back to a hardcoded home pose and sends duplicate stop commands |

### Execution Flow

```
ReadRegionCenters → for each center:
  ├─ HasNextCenterPoint?
  ├─ NavToIndexedPoseClient   (navigate to region)
  ├─ FindObject               (rotate + debounce until object found)
  ├─ ApproachObject           (align + approach + blind extension)
  ├─ ExecuteArmSequence × 2   (grab + retract)
  │   └─ [local loop] Find → Approach → Arm × 2  (opportunistic local pickups)
  ├─ IncrementIndex
  └─ ReturnToStartPosition
```

The Behavior Tree uses `KeepRunningUntilFailure` idioms to implement a retry-until-exhausted pattern: each region center is visited once for a "global" find-approach-grab sequence, followed by a local re-search loop that opportunistically picks up remaining visible objects before moving to the next region.

### Inter-Process Communication

Python nodes expose a standard Trigger service (`~/start`) and publish status on `~/status`. C++ BT action nodes call the service to start the operation, then poll the status topic for completion (`SUCCESS` / `FAILURE`). This decouples the BT engine from long-running perception/motion tasks.

### Map Partitioning

`map_partitioner.py` provides an offline pipeline for dividing an occupancy grid into reachable regions:

1. **Preprocess** — inflate obstacles by robot radius, extract the home-connected free-space component
2. **Seed detection** — local maxima of the distance transform (interior peaks) serve as region seeds
3. **Voronoi partition** — marker-based watershed on the inverted distance transform
4. **Greedy merge** — iteratively merge adjacent regions by an area/distance cost function, with an optional max-radius hard constraint
5. **TSP routing** — nearest-neighbor greedy tour from home through all reachable region centers

Output: a color-coded partition image and a `region_centers.txt` file consumed by `ReadRegionCenters` at runtime.

---

## Environment & Dependencies

### Required

| Component | Version / Notes |
|-----------|-----------------|
| Ubuntu | 22.04 (Jammy) |
| ROS 2 | Humble Hawksbill |
| Python | ≥ 3.10 |
| C++ Standard | C++17 |
| BehaviorTree.CPP | 4.x |
| Nav2 | Humble |
| Ultralytics YOLO | `pip install ultralytics` |
| OpenCV | `python3-opencv` or `pip install opencv-python` |
| scikit-image | `pip install scikit-image` |
| SciPy | `pip install scipy` |
| PyYAML | `pip install pyyaml` |
| RoboMaster SDK | If there is a version conflict, please refer to[Ronomaster-SDK-Ultra](https://github.com/RamessesN/Robomaster-SDK-Ultra) |
| Robomaster ROS | Used to bridge the gap between SDK and Ros | 
|YDlidar SDK and YDlidar Ros2 Driver| Used to drive ydlidar|

### Install Dependencies

```bash
# ROS 2 Humble (full desktop install recommended)
# https://docs.ros.org/en/humble/Installation.html

# Python dependencies
pip install ultralytics scikit-image scipy opencv-python pyyaml

# BehaviorTree.CPP V4
#https://github.com/BehaviorTree/BehaviorTree.CPP

# Nav2, slam toolbox

```

---

## Build & Run

### Prepare YOLO Weights
Place your trained YOLOv8 model weights in the `model` directory before launching the vision node.

### Build

```bash
cd ~/brickpick_ws
colcon build --packages-select brickpick --symlink-install
source install/setup.bash
```

The `--symlink-install` flag allows Python scripts to be edited without re-building.

### Preprocess

Run 
```bash
ros2 run brickpick odom_2d_filter.py
```
to create odom_2d.

Please start the ydlidar and connect the robomaster ep, and doing mapping work first.
```bash
ros2 launch robomaster_ros main.launch model:=ep conn_type:=sta
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false slam_params_file:=/home/nvidia/brickpick_ws/src/brickpick/config/mapper_params_online_async.yaml
ros2 run brickpick teleop_keyboard_node.py
```
After that run 
```bash
python3 brickpick/map_partitioner.py
```
to partion your map.

Then please start the localization and navigation services. 
```bash
ros2 launch nav2_bringup localization_launch.py \
map:=/home/nvidia/map/livingroom_clean_12.yaml \
params_file:=/home/nvidia/brickpick_ws/src/brickpick/config/nav2_params.yaml \
use_sim_time:=false

ros2 launch nav2_bringup navigation_launch.py \
params_file:=/home/nvidia/brickpick_ws/src/brickpick/config/nav2_params.yaml \
use_sim_time:=false
```

### Run the Full Pipeline

```bash
ros2 launch brickpick main.launch.py
```

This launches: `vision_node`, `find_node`, `approach_node`, `arm_preset_node`, and `bt_executor`.

### Run Individual Components

```bash
# Vision only
ros2 run brickpick vision_node.py --ros-args -p device:="cpu"

# Manual teleop
ros2 run brickpick teleop_keyboard_node.py

# Image capture for dataset
ros2 run brickpick image_capture_node.py --ros-args -p save_dir:="./dataset"

# Offline map partitioning
python3 brickpick/map_partitioner.py
```

### Run with a Custom Behavior Tree

```bash
ros2 launch brickpick main.launch.py bt_xml_path:=/path/to/custom_tree.xml
```

Two tree variants are included:

- `config/brickpick_tree.xml` — full multi-region traversal with local pickup loops
- `config/simple_pick_tree.xml` — single find-approach-grab cycle for debugging

---

## Configuration

All runtime parameters are in `config/`:

| File | Node | Key Parameters |
|------|------|----------------|
| `vision_params.yaml` | `brickpick_vision` | `model_path`, `device`, `conf_threshold`, `camera_topic` |
| `approach_params.yaml` | `approach_node` | `target_x_offset`, `yaw_kp`, `forward_speed`, `stop_y_threshold` |
| `arm_presets.yaml` | `arm_preset_node` | Preset positions (`home`, `forward`, `down`, `backward`), joint limits |
| `nav2_params.yaml` | Nav2 | Planner, controller, BT navigator configuration |
| `mapper_params_online_async.yaml` | slam_toolbox | SLAM parameters for online mapping |

---

## Directory Structure

```
BrickPick/
├── brickpick/               # Python nodes
│   ├── find_node.py
│   ├── vision_node.py
│   ├── approach_node.py
│   ├── arm_preset_node.py
│   ├── odom_2d_filter.py
│   ├── teleop_keyboard_node.py
│   ├── image_capture_node.py
│   ├── map_partitioner.py
│   └── test.py
├── src/                     # C++ Behavior Tree engine & custom nodes
│   ├── bt_executor.cpp
│   ├── bt_nodes_plugin.cpp
│   ├── find_action.cpp
│   ├── approach_action.cpp
│   ├── arm_action.cpp
│   ├── nav_to_indexed_pose_client.cpp
│   ├── reading_region.cpp
│   ├── has_next_center_point.cpp
│   ├── increment_index.cpp
│   └── return_to_start_position.cpp
├── include/brickpick/       # C++ headers
├── config/                  # Parameter YAML files & BT XML trees
├── launch/                  # ROS 2 launch files
├── map/                     # Occupancy grid maps (.pgm/.yaml)
├── model/                   # YOLO model weights (.pt)
├── sim/                     # Map generation & simulation utilities
├── CMakeLists.txt
└── package.xml
```

---

## Developing

See our developing process in [here](https://foremost-garage-64a.notion.site/33f3024d44438060a0d1fd7c040ab3d0?pvs=74)

---

## License

Apache 2.0 — see [LICENSE](LICENSE).

</details>

<details>
<summary><a id="chinese"><b>🇨🇳 中文</b></a> &nbsp;|&nbsp; <a href="#"><b>🇬🇧 English</b></a></summary>

# BrickPick
<img width="400" height="225" alt="4613_1779098093-ezgif com-optimize" src="https://github.com/user-attachments/assets/d433e084-e983-4fe5-bb63-a5573a930ec6" />

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.10-3776AB?logo=python)](https://www.python.org/)
[![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?logo=c%2B%2B)](https://isocpp.org/)
[![BehaviorTree.CPP](https://img.shields.io/badge/BT.CPP-4.x-green)](https://www.behaviortree.dev/)
[![YOLO](https://img.shields.io/badge/YOLO-11-orange?logo=ultralytics)](https://docs.ultralytics.com/)
[![License](https://img.shields.io/badge/License-Apache_2.0-lightgrey)](LICENSE)

基于 RoboMaster EP 移动机械臂的自主积木抓取系统：视觉引导导航、带盲区接管的物体逼近、机械臂序列动作，全部由行为树统一编排调度。

---

## 系统架构

### 核心 ROS 2 节点

| 节点 | 语言 | 功能 |
|------|------|------|
| `brickpick_vision` | Python | 在相机图像上运行 YOLO 推理；发布 `Detection2DArray` 和标注后的调试画面 |
| `find_node` | Python | 原地旋转 + 多帧防抖确认；连续多帧检测到目标后才上报 SUCCESS，防止误触发 |
| `approach_node` | Python | 状态机（`IDLE → ALIGN → APPROACH → EXTEND → DONE`）：偏航对准、前进靠近、目标进入下视盲区后盲驶延长 |
| `arm_preset_node` | Python | 机械臂预设位姿序列控制器；使用可重入回调组，通过 `MoveArm` Action 执行可配置的位置序列 |
| `odom_2d_filter` | Python | 将 6 自由度里程计投影为 2D 变换（`odom_2d → base_link`），剥离 Z、roll、pitch |
| `teleop_keyboard_node` | Python | 增量式键盘遥控（TurtleBot3 风格），用于手动测试 |
| `image_capture_node` | Python | 按键触发图像抓拍，用于数据集采集 |
| `brickpick_bt_executor` | C++ | 行为树引擎；加载 XML 行为树、注册自定义节点、以 50 Hz 频率驱动任务循环 |

### 行为树自定义节点

| 节点 | 类型 | 说明 |
|------|------|------|
| `ReadRegionCenters` | Action | 解析 `region_centers.txt` 为可通过端口访问的坐标向量 |
| `HasNextCenterPoint` | Condition | 检查 `current_index` 是否仍在区域向量范围内 |
| `NavToIndexedPoseClient` | Action | 向 Nav2 发送指定索引区域中心的 `NavigateToPose` 目标 |
| `FindObject` | Action | 通过 Trigger 服务调用 `find_node`，轮询状态话题等待完成 |
| `ApproachObject` | Action | 通过 Trigger 服务调用 `approach_node`，轮询状态话题等待完成 |
| `ExecuteArmSequence` | Action | 通过 Trigger 服务调用 `arm_preset_node`，轮询状态话题等待完成 |
| `IncrementIndex` | Action | 双向端口：读取 `current_index`，写入 `current_index + 1` |
| `ReturnToStartPosition` | Action | 导航回预设原点并发送重复刹车指令确保停机 |

### 执行流程

```
ReadRegionCenters → 遍历每个区域中心:
  ├─ HasNextCenterPoint?
  ├─ NavToIndexedPoseClient   (导航到目标区域)
  ├─ FindObject               (旋转搜索 + 防抖确认)
  ├─ ApproachObject           (对准 + 靠近 + 盲区接管)
  ├─ ExecuteArmSequence × 2   (抓取 + 回收)
  │   └─ [局部循环] Find → Approach → Arm × 2  (机会性局部捡取)
  ├─ IncrementIndex
  └─ ReturnToStartPosition
```

行为树使用 `KeepRunningUntilFailure` 实现"重试至耗尽"模式：每个区域中心先执行一次全局 Find→Approach→Grab，然后进入局部搜索循环，尽可能捡完当前视野中的所有目标，再移动到下一个区域。

### 进程间通信

每个 Python 节点暴露 Trigger 服务 (`~/start`) 并在 `~/status` 话题上发布状态。C++ 行为树节点通过调用服务启动操作，然后轮询状态话题等待完成信号（`SUCCESS` / `FAILURE`），从而将行为树引擎与长时间运行的感知/运动任务解耦。

### 地图分区

`map_partitioner.py` 提供离线地图分区流水线，将占据栅格地图划分为可到达的区域：

1. **预处理** — 按机器人半径膨胀障碍物，提取与 home 连通的自由空间
2. **种子检测** — 距离变换的局部极大值（内部峰值）作为区域种子点
3. **Voronoi 分区** — 在反转距离变换上进行基于标记的分水岭分割
4. **贪心合并** — 通过面积/距离代价函数迭代合并相邻区域，支持最大半径硬约束
5. **TSP 路径规划** — 从 home 出发的最近邻贪心巡游

输出：一张着色分区图和一份 `region_centers.txt` 文件，供 `ReadRegionCenters` 在运行时读取。

---

## 环境与依赖

### 必需

| 组件 | 版本 / 备注 |
|------|-------------|
| Ubuntu | 22.04 (Jammy) |
| ROS 2 | Humble Hawksbill |
| Python | ≥ 3.10 |
| C++ 标准 | C++17 |
| BehaviorTree.CPP | 4.x |
| Nav2 | Humble |
| Ultralytics YOLO | `pip install ultralytics` |
| OpenCV | `python3-opencv` 或 `pip install opencv-python` |
| scikit-image | `pip install scikit-image` |
| SciPy | `pip install scipy` |
| PyYAML | `pip install pyyaml` |
| RoboMaster SDK | 如果遇到版本冲突，请参考[Ronomaster-SDK-Ultra](https://github.com/RamessesN/Robomaster-SDK-Ultra) |
| Robomaster ROS | 用于弥合SDK和Ros | 
|YDlidar SDK and YDlidar Ros2 Driver| 用于驱动ydlidar|

### 安装依赖

```bash
# ROS 2 Humble（推荐完整桌面安装）
# https://docs.ros.org/en/humble/Installation.html

# Python 依赖
pip install ultralytics scikit-image scipy opencv-python pyyaml

# BehaviorTree.CPP V4
#https://github.com/BehaviorTree/BehaviorTree.CPP

# Nav2, slam toolbox
```

---

## 编译与运行

### 准备 YOLO 权重文件
在启动视觉节点前，请将训练好的 YOLOv8 权重文件放置在 `model` 目录下。

### 编译

```bash
cd ~/brickpick_ws
colcon build --packages-select brickpick --symlink-install
source install/setup.bash
```

`--symlink-install` 选项允许在无需重新编译的情况下直接修改 Python 脚本。

### 前置工作

运行 
```bash
ros2 run brickpick odom_2d_filter.py
```
创建odom_2d

请开启雷达，并连接robomaster EP. 首先请完成建图工作
```bash
ros2 launch robomaster_ros main.launch model:=ep conn_type:=sta
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=false slam_params_file:=/home/nvidia/brickpick_ws/src/brickpick/config/mapper_params_online_async.yaml
ros2 run brickpick teleop_keyboard_node.py
```
运行
```bash
python3 brickpick/map_partitioner.py
```
来分割地图

请开启定位和导航服务 
```bash
ros2 launch nav2_bringup localization_launch.py \
map:=/home/nvidia/map/livingroom_clean_12.yaml \
params_file:=/home/nvidia/brickpick_ws/src/brickpick/config/nav2_params.yaml \
use_sim_time:=false

ros2 launch nav2_bringup navigation_launch.py \
params_file:=/home/nvidia/brickpick_ws/src/brickpick/config/nav2_params.yaml \
use_sim_time:=false
```


### 运行完整流水线

```bash
ros2 launch brickpick main.launch.py
```

此命令启动：`vision_node`、`find_node`、`approach_node`、`arm_preset_node` 和 `bt_executor`。

### 单独运行组件

```bash
# 仅运行视觉
ros2 run brickpick vision_node.py --ros-args -p device:="cpu"

# 手动遥控
ros2 run brickpick teleop_keyboard_node.py

# 数据集图像采集
ros2 run brickpick image_capture_node.py --ros-args -p save_dir:="./dataset"

# 离线地图分区
python3 brickpick/map_partitioner.py
```

### 使用自定义行为树

```bash
ros2 launch brickpick main.launch.py bt_xml_path:=/path/to/custom_tree.xml
```

包含两种行为树变体：

- `config/brickpick_tree.xml` — 完整多区域遍历，含局部捡取循环
- `config/simple_pick_tree.xml` — 单次 Find→Approach→Grab 循环，用于调试

---

## 配置

所有运行时参数位于 `config/` 目录：

| 文件 | 对应节点 | 关键参数 |
|------|----------|----------|
| `vision_params.yaml` | `brickpick_vision` | `model_path`、`device`、`conf_threshold`、`camera_topic` |
| `approach_params.yaml` | `approach_node` | `target_x_offset`、`yaw_kp`、`forward_speed`、`stop_y_threshold` |
| `arm_presets.yaml` | `arm_preset_node` | 预设位姿（`home`、`forward`、`down`、`backward`）、关节限位 |
| `nav2_params.yaml` | Nav2 | 规划器、控制器、行为树导航器配置 |
| `mapper_params_online_async.yaml` | slam_toolbox | 在线建图 SLAM 参数 |

---

## 目录结构

```
BrickPick/
├── brickpick/               # Python 节点
│   ├── find_node.py
│   ├── vision_node.py
│   ├── approach_node.py
│   ├── arm_preset_node.py
│   ├── odom_2d_filter.py
│   ├── teleop_keyboard_node.py
│   ├── image_capture_node.py
│   ├── map_partitioner.py
│   └── test.py
├── src/                     # C++ 行为树引擎与自定义节点
│   ├── bt_executor.cpp
│   ├── bt_nodes_plugin.cpp
│   ├── find_action.cpp
│   ├── approach_action.cpp
│   ├── arm_action.cpp
│   ├── nav_to_indexed_pose_client.cpp
│   ├── reading_region.cpp
│   ├── has_next_center_point.cpp
│   ├── increment_index.cpp
│   └── return_to_start_position.cpp
├── include/brickpick/       # C++ 头文件
├── config/                  # 参数 YAML 文件与行为树 XML
├── launch/                  # ROS 2 启动文件
├── map/                     # 占据栅格地图（.pgm / .yaml）
├── model/                   # YOLO 模型权重（.pt）
├── sim/                     # 地图生成与仿真工具
├── CMakeLists.txt
└── package.xml
```

---

## 开发流程

可以在[这里](https://foremost-garage-64a.notion.site/33f3024d44438060a0d1fd7c040ab3d0?pvs=74)看到我们的开发流程

---

## License

Apache 2.0 — 详见 [LICENSE](LICENSE)。

</details>
