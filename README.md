# Flexiv Rizon 4s 视觉抓取实验工作区

ROS 2 Humble 工作区，在 Flexiv 官方 `flexiv_ros2` 的基础上，实现了一套**腕部相机 + AprilTag 的
单物体抓取 demo**，以及配套的**手眼标定工具**。

目标是跑通一条可复现的链路：机械臂到观察位 → 腕部 D415 识别贴标的 40 mm 立方体 → MoveIt 规划 →
Grav 夹爪力控抓取 → 抬起 → 放到固定点 → 松开 → 退回，全部由一个 ROS 2 action 触发。

| 项 | 内容 |
| --- | --- |
| 机器人 | Flexiv Rizon 4s（SN `Rizon4s-064230`） |
| 夹爪 | Flexiv Grav（`Flexiv-GN01`） |
| 相机 | Intel RealSense D415，eye-in-hand 装在腕部 |
| 软件 | Ubuntu 22.04.5 · ROS 2 Humble · MoveIt 2.5.10 |
| 抓取目标 | 40×40×40 mm 立方体，20 g，六面贴 26 mm AprilTag **25h9** |

---

## 仓库里有什么

本仓库**只包含自己写的代码和文档**，上游 Flexiv 包通过 `dependencies.repos` 按版本拉取。

```text
flexiv_ros2_ws/
├── src/
│   ├── flexiv_grasp_demo/       # 抓取 demo：C++ 任务节点 + Python 感知节点 + launch/配置/自检脚本
│   ├── flexiv_handeye_calib/    # 手眼标定：采样、求解、精度验证（纯 Python，无 cv_bridge）
│   ├── flexiv_ros2/             # ← 上游，vcs 拉取
│   ├── flexiv_description/      # ← 上游，vcs 拉取
│   └── flexiv_rdk/              # ← 上游，vcs 拉取
├── docs/
│   ├── grasp_demo_requirements.md   # 抓取 demo 需求、架构、接口、里程碑
│   ├── handeye_calibration.md       # 手眼标定方案、误差预算与七层验证方法
│   ├── handeye_runbook.md           # 手眼标定现场操作手册
│   ├── m1_bringup.md                # M1 真机联调步骤
│   └── m2_vision_check.md           # M2 视觉静态验证方法
├── dependencies.repos           # 上游依赖版本锁定
└── README.md
```

不入库的内容：`build/ install/ log/`（编译产物）、`.cache/`、`.ros_home/`（本机运行状态）、
`experiment_report/`（实验记录与对话素材，本地单独保存）。

---

## 上游依赖

版本锁定在 [`dependencies.repos`](dependencies.repos)：

| 包 | 上游 | 版本 |
| --- | --- | --- |
| `flexiv_ros2` | [flexivrobotics/flexiv_ros2](https://github.com/flexivrobotics/flexiv_ros2) | `humble-v1.9.3` |
| `flexiv_description` | [flexivrobotics/flexiv_description](https://github.com/flexivrobotics/flexiv_description) | `humble-v3.0` |
| `flexiv_rdk` | [flexivrobotics/flexiv_rdk](https://github.com/flexivrobotics/flexiv_rdk) | `v1.9.3` |

```bash
git clone https://github.com/TronK1ng/flexiv_ros2_ws.git
cd flexiv_ros2_ws
vcs import src < dependencies.repos   # 需要 python3-vcstool
```

`flexiv_rdk` 自带 `COLCON_IGNORE`，不参与 colcon 编译，但需要先单独编译安装它的 C++ 库。

---

## 编译

```bash
# 1) 编译 RDK，装到 ~/flexiv_install
cd src/flexiv_rdk
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
cmake --install build --prefix "$HOME/flexiv_install"
cd ../..

# 2) 编译上游 flexiv_ros2（这一步需要 RDK 前缀）
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-up-to flexiv_bringup flexiv_gripper flexiv_moveit_config \
  --cmake-args -DCMAKE_PREFIX_PATH="$HOME/flexiv_install"

# 3) 编译本仓库的两个包（注意：不要带 RDK 前缀）
colcon build --symlink-install \
  --packages-select flexiv_grasp_demo flexiv_handeye_calib
```

> **为什么第 3 步不能带 `-DCMAKE_PREFIX_PATH`**：RDK 的安装目录里带了 Boost 1.83，而系统是
> Boost 1.74。`flexiv_grasp_demo` 不依赖 RDK 库，一旦带上这个前缀，MoveIt 会去链接 Boost 1.83
> 并与系统版本冲突。

---

## 快速验证（无真机）

不需要机器人、不需要相机，用 fake hardware 跑完整状态机：

```bash
bash src/flexiv_grasp_demo/test/smoke_fake_hardware.sh
```

脚本会拉起整套 fake 栈（`ROS_DOMAIN_ID=77` 隔离）、等 action server、发一个 goal，
按退出码给出成败。预期走完
`OBSERVE → DETECT → APPROACH → GRASP → LIFT → PLACE → RELEASE → RETREAT`，返回 `success: true`。

---

## 真机流程

| 阶段 | 内容 | 文档 |
| --- | --- | --- |
| M1 | 驱动 + MoveIt + 夹爪空载联调，不靠近物体 | [`docs/m1_bringup.md`](docs/m1_bringup.md) |
| M2 | 视觉链路静态验证：同一静止方块在多个姿态下 base 系位置一致 | [`docs/m2_vision_check.md`](docs/m2_vision_check.md) |
| M3 | 真机抓取闭环 | [`docs/grasp_demo_requirements.md`](docs/grasp_demo_requirements.md) |

一条命令起全部（默认参数已按实验室这台机器固化）：

```bash
ros2 launch flexiv_grasp_demo grasp_demo.launch.py
ros2 action send_goal /grasp_demo/pick_place flexiv_grasp_demo/action/PickPlace \
  "{object_id: '7'}" --feedback
```

---

## 当前进度

| 里程碑 | 内容 | 状态 |
| --- | --- | --- |
| M0 | 建包、接口、launch、状态机在 fake hardware 下空跑 | ✅ 已通过 |
| M1 | 真机空载运动 + 夹爪联调 | ✅ 已完成（2026-10-30） |
| M2 | 视觉静态验证（同一静止方块，多姿态位置一致性） | 🟡 横向 x/y 跨度 1.4–2.8 mm 合格；深度 z 跨度 13 mm 偏大，已加 3 帧中位数滤波 |
| M3 | 真机抓取闭环 | 🟡 首次抓取成功（2026-10-30），重复性验证待做 |
| M4 | 放置 + 10 次重复性统计 | ⬜ 未开始 |

手眼标定结果（24 组样本，`PARK` 解法）：

```text
Rizon4s-064230_flange -> camera_link
xyz  = [-0.081268, -0.009562, 0.022950] m
quat = [-0.004745, -0.511938, 0.003057, 0.859004]

旋转残差 0.63° rms / 1.69° max
平移残差 1.63 mm rms / 3.09 mm max
留一交叉验证 1.73 mm rms
独立位姿复现 4.16 mm rms（判据 ACCEPTABLE）
```

---

## 文档索引

| 文档 | 内容 |
| --- | --- |
| [`docs/grasp_demo_requirements.md`](docs/grasp_demo_requirements.md) | 需求与设计确认单：场景、架构、TF 约定、接口、参数、状态机、里程碑、风险 |
| [`docs/handeye_calibration.md`](docs/handeye_calibration.md) | 手眼标定：开源方案对比、标定方程、七层精度验证、误差预算、排错 |
| [`docs/handeye_runbook.md`](docs/handeye_runbook.md) | 手眼标定现场操作手册（一步步照着做） |
| [`docs/m1_bringup.md`](docs/m1_bringup.md) | M1 真机联调步骤与中止条件 |
| [`docs/m2_vision_check.md`](docs/m2_vision_check.md) | M2 视觉静态验证：为什么不用卷尺、三姿态判据 |
| [`src/flexiv_grasp_demo/README.md`](src/flexiv_grasp_demo/README.md) | 抓取 demo 的构建、运行、参数、错误码、已知限制 |
| [`src/flexiv_handeye_calib/README.md`](src/flexiv_handeye_calib/README.md) | 标定工具的命令、服务、参数 |

---

## 复现时的坑

1. **OpenCV 版本**：`cv2.aruco.DICT_APRILTAG_25H9` 需要 OpenCV ≥ 4.7。apt 的 4.5.4 没有这个字典，
   本机用 `~/.local` 的 pip 包（4.13）。同进程里同时加载两份 OpenCV 会崩，所以两个包都**不用 `cv_bridge`**。
2. **Boost 冲突**：见上文编译第 3 步。
3. **ROS 域隔离**：`smoke_fake_hardware.sh` 用 `ROS_DOMAIN_ID=77`，避免和真机栈互相干扰；
   手工对比时注意两个终端要在同一个 RMW / domain 下。
4. **`grav_tcp` 与示教器 TCP**：已确认重合。若换了工具，先按
   [`src/flexiv_grasp_demo/README.md`](src/flexiv_grasp_demo/README.md) 的 M1 核对清单验一遍。
5. **放置点高度语义**：示教器 `z=-0.05021` 是空爪指尖触桌面，`place_pose` 存的是方块中心
   （桌面 + 20 mm），别直接抄示教器读数。

---

## 致谢

上游 [Flexiv RDK](https://github.com/flexivrobotics/flexiv_rdk)、
[flexiv_ros2](https://github.com/flexivrobotics/flexiv_ros2)、
[flexiv_description](https://github.com/flexivrobotics/flexiv_description)，
以及 OpenCV 的 `calibrateHandEye` 与 ArUco/AprilTag 实现。
