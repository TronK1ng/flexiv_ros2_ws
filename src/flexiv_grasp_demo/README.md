# flexiv_grasp_demo

基于 `flexiv_ros2` 的单物体抓取 demo：腕部 D415 识别 AprilTag 25h9 → MoveIt 规划 → Grav 夹爪力控抓取 → 抬起 → 放到固定点 → 松开。

目标物体：**40×40×40 mm 立方体，20 g**，六面贴 26 mm AprilTag 25h9。

当前状态：**M0 已通过**（fake hardware + mock 检测跑完整流程，action 返回 `success: true`）。M1 及之后需要在真机上联调，见 `docs/grasp_demo_requirements.md`。

## 依赖

- ROS 2 Humble、MoveIt 2、`flexiv_msgs`、`flexiv_bringup`、`flexiv_gripper`
- Python：`numpy`、OpenCV **4.13**（需要 `cv2.aruco.DICT_APRILTAG_25H9`；本机由 `~/.local` 的 pip 包提供，apt 的 4.5.4 不满足）
- 不使用 `cv_bridge`：系统里同时存在两份 OpenCV，同进程加载会冲突

## 构建

```bash
cd ~/tianJiZhiYuan/flexiv_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select flexiv_grasp_demo
source install/setup.bash
```

注意：这个包**不要**加 `-DCMAKE_PREFIX_PATH=$HOME/flexiv_install`，否则 MoveIt 会去找 Boost 1.83 并和系统的 1.74 冲突。本包不依赖 RDK 库。

## 运行

### 1. 无真机（M0 冒烟测试）

```bash
bash src/flexiv_grasp_demo/test/smoke_fake_hardware.sh
```

脚本会起整套 fake 栈、等 action server、发一个 goal、打印结果并以退出码表示成败。内部用 `ROS_DOMAIN_ID=77` 隔离，避免和你已开的栈互相干扰。

手动跑：

```bash
ros2 launch flexiv_grasp_demo grasp_demo.launch.py \
  robot_sn:=Rizon4s-123456 use_fake_hardware:=true detect_mode:=mock start_rviz:=false
```

### 2. 真机

前提：机器人已启用 RDK server、网线连好、机器人 operational。真机 SN 是 `Rizon4s-064230`。

一条命令起全部（相机 + 机械臂 + MoveIt + 手眼 TF + 感知 + 任务节点）：

```bash
ros2 launch flexiv_grasp_demo grasp_demo.launch.py
```

默认参数已经固化成实验室这台机器的配置，不用再传：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `robot_sn` | `Rizon4s-064230` | 真机序列号 |
| `robot_type` | `Rizon4s` | 机型 |
| `velocity_scaling` | `0.05` | 5% 速度 |
| `acceleration_scaling` | `0.05` | 5% 加速度 |
| `grasp_force` | `10.0` | 抓取力 10 N |
| `start_camera` | `true` | 同时启动 RealSense |
| `camera_color_profile` | `1280x720x30` | 彩色流分辨率 |

相机也可以单独起（`start_camera:=false`），但要保证彩色流话题与配置一致。

手眼外参由 `handeye_tf_publisher.py` 自动发布成静态 TF：按 `./.cache/handeye/` →
`~/.cache/handeye/` → 包内 `config/handeye_calibration.yaml` 的顺序找文件，也可以用
`handeye_yaml:=<path>` 显式指定。当前标定：

```
Rizon4s-064230_flange -> camera_link
xyz = [-0.081268, -0.009562, 0.022950] m
quat = [-0.004745, -0.511938, 0.003057, 0.859004]
质量：24 组样本，平移 1.63 mm rms，旋转 0.63° rms
```

重新标定后把新 yaml 拷到上面任一位置即可，不用改 launch。

> 注意：相机不是正对工具轴——`camera_link` 的 x 轴与 flange z 轴夹角 **28.4°**。
> 工具竖直朝下时，相机视野中心偏离 TCP 竖直下方约 5–6 cm（朝 -x）。这在 D415 的
> 视场里（0.3 m 处半视场约 21 cm），属正常现象，不是装歪了。

### 3. 发 goal

```bash
ros2 action send_goal /grasp_demo/pick_place flexiv_grasp_demo/action/PickPlace \
  "{object_id: '7'}" --feedback
```

字段都可以省略（`{}` 即为全部用 YAML 默认值）；`object_id` 为空表示取视野里面积最大的标记。

## 节点

| 节点 | 说明 |
| --- | --- |
| `grasp_task` | 状态机：OBSERVE → DETECT → APPROACH → GRASP → LIFT → PLACE → RELEASE → RETREAT |
| `grasp_perception` | AprilTag 检测 + PnP + TF 到 base，按需响应 `/grasp_demo/detect` |
| `flexiv_gripper_node` | 真机时由 `flexiv_bringup` 启动；fake 时由本包的 `fake_gripper_server.py` 顶替 |

## 接口

- Action `/grasp_demo/pick_place`（`flexiv_grasp_demo/action/PickPlace`）
- Service `/grasp_demo/detect`（`flexiv_grasp_demo/srv/Detect`）→ `GraspTarget[]`
- 复用：`/flexiv_gripper_node/{move,grasp}`、`/flexiv_gripper_node/stop`

## M1 现场核对清单

上真机前先花五分钟确认两件事，否则后面所有点位都会偏。

**1. 确认 `grav_tcp` 和示教器 / RDK 的 TCP 是同一个点**

```bash
# 终端 A：TF 给出的 TCP 位姿（MoveIt 规划用的就是这个）
ros2 run tf2_ros tf2_echo Rizon4s-064230_base_link Rizon4s-064230_grav_tcp

# 终端 B：RDK 直接上报的 TCP，等于示教器上显示的数值
ros2 topic echo /Rizon4s-064230/tcp_pose
```

两边的位置差就是 `grav_tcp` 与实际工具 TCP 的偏差：如果差了 `d`，把 `grasp_offset` 再减 `d`（顶抓时沿法向向下为正）。

**2. 放置点的高度约定**

示教器读数 `z = -0.05021` 是**空爪指尖触桌面**读到的，也就是桌面高度。
`place_pose_xyz` 里存的是 **TCP = 方块中心 = 桌面 + 20 mm**（`-0.03021`），和抓取时的约定一致
（`grasp_offset = -0.02` 同样把 TCP 放在方块中心）。改放置点时记得保持这个约定，别直接抄示教器读数。

## 参数

全部在 `config/grasp_demo.yaml`，几个关键的：

| 参数 | 含义 |
| --- | --- |
| `observe_pose_xyz/rpy` | 拍照位姿，必须示教；物体要落在相机视野里 |
| `place_mode` | `relative`（放到抓取点旁边）或 `fixed`（放到固定点） |
| `place_offset_from_grasp` | relative 模式下的偏移，base 系；`[0, 0.10, 0]` = 抓取点左侧 10 cm（+y 为机器人左手边） |
| `place_pose_xyz/rpy` | fixed 模式的放置点；relative 模式下只取它的 z（放下高度） |
| `grasp_height_mode` | `table`（默认，用桌面+方块高度算抓取高度）或 `marker`（信检测的 z） |
| `object_height` | 方块高度，`table` 模式下决定抓取深度 |
| `table_z` / `min_tcp_clearance` | 桌面高度与最低安全间隙（20 mm） |
| `grasp_offset` | 沿标记法向的补偿；`table` 模式下只影响 x/y |
| `approach_height` / `lift_height` | 预抓取/抬升距离 |
| `grasp_force` / `open_width` / `close_width` | 夹爪力与开合宽度；当前 10 N / 70 mm / 10 mm（20 g 立方体，开口留 30 mm 余量） |
| `velocity_scaling` | 真机建议 0.1 起步 |
| `tag_family` / `tag_size` | `25h9` / `0.026` |

## 错误码

| 码 | 含义 |
| --- | --- |
| `PRECHECK_FAILED` | MoveIt / 夹爪 / 服务未就绪 |
| `DETECT_FAILED` | 无标记、帧过期、服务超时 |
| `PLAN_FAILED` | 规划失败或笛卡尔路径覆盖率不足 |
| `EXEC_FAILED` | 控制器拒绝轨迹 |
| `GRASP_FAILED` | 夹爪动作失败 |
| `CANCELED` | 外部取消 |

## 已知限制

- 开环抓取：观察位拍一次照，不做视觉伺服。
- 取消只在阶段边界生效，不会打断正在执行的轨迹。
- `measured_width` 取自夹爪 feedback，fake 下是模拟值。
- 感知节点暂不使用深度流。
