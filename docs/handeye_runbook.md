# 手眼标定运行手册（Flexiv Rizon 4s · eye-in-hand）

一页跑完全流程。细节原理见 [`handeye_calibration.md`](handeye_calibration.md)。

---

## 0. 系统组成

```
 T1  rizon_moveit.launch.py ── robot_state_publisher ──► tf: {sn}_base_link → {sn}_flange
     └ ros2_control_node（RDK）─► /rizon_arm_controller/follow_joint_trajectory
     └ move_group ──► /compute_ik

 T2  realsense2_camera ──► /camera/camera/color/image_raw
                       └► /camera/camera/color/camera_info
                       └► tf: camera_link → camera_color_optical_frame

 T3  handeye_calibrator（本包）
       订阅 图像 / 内参 / tf
       提供 /handeye_calibrator/{capture,solve,save,reset,status}
       输出 ~/.ros/flexiv_handeye/{samples.json, handeye_calibration.yaml}
       发布 tf: {sn}_flange → camera_link（publish_tf:=true 时）

 T4（可选）auto_move：调 /compute_ik + JTC 自动摆位，再调 capture
 T5（可选）handeye_validator：用已发布 tf 复算标靶位姿，评估重复性
```

| 可执行文件 | 作用 |
| --- | --- |
| `doctor` | 启动前自检：相机、内参、tf、标靶、每格像素 |
| `calibrate` | 采样 + 求解 + 写 YAML + 发 tf |
| `auto_move` | 自动摆 14 个姿态并采集（可选） |
| `pose_delta` | 实时显示相对参考位姿转了多少度 |
| `validate` | 独立位姿复现，评估端到端重复性 |
| `handeye_solve` | 离线复算/换解法（读 samples.json） |

---

## 1. 前置条件（缺一不可）

| 项 | 检查 |
| --- | --- |
| 机器人模式 **Auto (Remote)** | motion bar 开关拨上 → 确认 → 选 AUTO (REMOTE)；Elements 关掉 |
| 急停释放、机器人已使能 | Elements 显示 Operational |
| 网络 | `ip a` 有机器人网段 IP；`ping <机器人IP>` 通 |
| 标靶 | 8×11 内角点、方格 6 mm、图案 54×72 mm；平放、胶带固定、中心点个铅笔点 |
| 相机 | 1280×720，**关自动曝光**，镜头干净，支架刚性 |
| 编译 | `colcon build --packages-select flexiv_handeye_calib` |

标靶距离：**0.30–0.35 m**（每格 ≥15 px）。

---

## 2. 阶段 A — 启动三个终端

```bash
# T1 机器人栈（robot_type 必须显式给 Rizon4s，默认是 Rizon4）
source ~/tianJiZhiYuan/flexiv_ros2_ws/install/setup.bash
ros2 launch flexiv_bringup rizon_moveit.launch.py \
    robot_sn:=Rizon4s-064230 robot_type:=Rizon4s \
    load_gripper:=false start_rviz:=true

# T2 相机
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=1280x720x30

# T3 标定节点
source ~/tianJiZhiYuan/flexiv_ros2_ws/install/setup.bash
ros2 run flexiv_handeye_calib calibrate --ros-args -p robot_sn:=Rizon4s-064230 \
    -p auto_capture:=true
```

成功标志：

```
[FlexivHardwareInterface]: Starting... please wait...
[resource_manager]: Successful 'activate' of hardware 'FlexivHardwareInterface'
[spawner]: Configured and activated rizon_arm_controller
```

```
target : chessboard 8x11 inner corners, 6.0 mm squares
auto_capture on: ...（auto_capture:=true 时）
>>> pose 1/17: stay at the observation pose (seed sample) ...
```

---

## 3. 阶段 B — 自检

```bash
ros2 run flexiv_handeye_calib doctor --ros-args -p robot_sn:=Rizon4s-064230 \
    -p target_type:=chessboard -p timeout_s:=3.0
```

必须全 PASS（一条 SKIP 正常）：

```
[PASS] camera image / info / intrinsics
[PASS] tf base <- flange
[PASS] tf camera <- optical
[PASS] target visible - ... ~31 px/square
[PASS] corner spacing - 31.0 px per square
[SKIP] tag pose in base - no flange -> camera_link published yet
```

`corner spacing` FAIL → 把相机移近；`tf` FAIL → 回阶段 A 检查机器人栈。

---

## 4. 阶段 C — 采样（二选一）

### C1 手动引导（先做这个）

在 RViz 里用 MotionPlanning 拖末端（速度 0.1），按 T3 打印的 `pose k/17` 依次摆姿态（第 1 个就是原地不动的基准位），每个停 1 秒，
`auto_capture:=true` 时自动采集（没开就手动调 capture）。手动补采：

```bash
ros2 service call /handeye_calibrator/capture std_srvs/srv/Trigger
ros2 service call /handeye_calibrator/status  std_srvs/srv/Trigger
```

不放心转了多少度，就开一个终端：

```bash
ros2 run flexiv_handeye_calib pose_delta --ros-args -p robot_sn:=Rizon4s-064230
```

### C2 自动摆位（可选）

```bash
# T3 换成：加 -p guided:=false
ros2 run flexiv_handeye_calib calibrate --ros-args -p robot_sn:=Rizon4s-064230 -p guided:=false

# T4 先试两个姿态
ros2 run flexiv_handeye_calib auto_move --ros-args -p robot_sn:=Rizon4s-064230 -p max_poses:=2

# 先把机械臂停在观察位（板在画面里、约 0.3 m），确认 auto_move 看得到
ros2 service call /auto_move/status std_srvs/srv/Trigger
#   success=True  ready, target at pixel (620, 355)   ← 这样才能开始
#   success=False the target must be fully visible at the start pose: ...

ros2 service call /auto_move/run   std_srvs/srv/Trigger
ros2 service call /auto_move/abort std_srvs/srv/Trigger   # 随时取消
```

`step_mode:=true`（默认）时**每次 `~/run` 只走一个姿态**：第一次会先采基准位、做两次 2 cm 探针，
再走第 1 个姿态；之后每调一次走下一个，日志会提示 `pose k/14 done - call ~/run for the next pose`。
想一次跑完就加 `-p step_mode:=false`。

采满 17 个（含基准位）后节点会**自动求解**、写 YAML、发 tf 并打印判读（`auto_solve_on_plan:=true`，默认开）。
想手动控制就设 `auto_solve_on_plan:=false`。

**采样质量门禁**（自动生效）：线速度 <0.05 m/s、角速度 <8 °/s、静止 0.6 s、单帧重投影 <3 px；
每采一帧与已有样本做一致性比较，>1.2° 立刻警告。

---

## 5. 阶段 D — 求解与判读

```bash
ros2 service call /handeye_calibrator/solve std_srvs/srv/Trigger
```

```
samples     : 16 used, 0 rejected
residual    : 0.9 mm rms / 1.8 mm max, 0.15 deg rms / 0.3 deg max
leave-one-out: 1.1 mm rms, 0.16 deg rms
rotation diversity: max 42.0 deg, mean 21.0 deg
consistency : 0.15 deg (max 0.9)  |  axis spread 45.0 deg
method spread : 0.5 mm, 0.2 deg
verdict     : GOOD
```

| 指标 | 可用 | 重做 |
| --- | --- | --- |
| `consistency` | < 0.3° | > 0.5°（拖影 / 板动了） |
| `residual` RMS | < 2 mm | > 5 mm |
| `leave-one-out` | < 3 mm | ≫ 拟合残差 |
| `axis spread` | > 20° | < 20°（只滚转） |
| `method spread` | < 2 mm | > 5 mm |

不合格：`reset` → 重采。输出在 `~/.ros/flexiv_handeye/handeye_calibration.yaml`。

---

## 6. 阶段 E — 验证

### E1 新位姿复现（量化）

**保持标定节点运行**（它负责发 `flange→camera_link`），然后：

```bash
ros2 launch flexiv_handeye_calib validate.launch.py robot_sn:=Rizon4s-064230 \
    target_type:=chessboard chessboard_cols:=8 chessboard_rows:=11 square_size:=0.006
```

摆 8–10 个**没用过**的新姿态，每个：

```bash
ros2 service call /handeye_validator/record std_srvs/srv/Trigger
```

全部录完后：

```bash
ros2 service call /handeye_validator/report std_srvs/srv/Trigger
```

`rms < 3 mm` 合格，`< 1.5 mm` 优秀。

### E2 物理点触（金标准）

1. 让视觉给出标靶中心 `P`（E1 报告或 `report` 输出）；
2. RViz 把 TCP 移到 `P` 正上方 1 cm，再沿工具 z 缓慢下降到刚好接触板面；
3. 量 TCP 尖端与铅笔点的横向偏差；
4. 换 5–8 个位置重复。

平均 < 3 mm、最大 < 5 mm 合格。均值偏大而离散小 → 系统性偏差，回查 `square_size` 与内参。

---

## 7. 阶段 F — 部署到抓取 demo

外参在 `~/.ros/flexiv_handeye/handeye_calibration.yaml`（xyz 米 / rpy 弧度 / 四元数 / 4×4）。

**方式 A**：保持 `calibrate` 节点运行（启动时自动重发 YAML 里的 tf），`publish_tf:=true`（默认）。

**方式 B**：自己发 static tf，此时标定节点必须 `publish_tf:=false`：

```bash
ros2 run tf2_ros static_transform_publisher \
    --x <x> --y <y> --z <z> --qx <qx> --qy <qy> --qz <qz> --qw <qw> \
    --frame-id Rizon4s-064230_flange --child-frame-id camera_link
```

同一个 `(parent, child)` 只能有一个发布者。之后起抓取 demo，确认 `grasp_perception` 能查到
`base ← camera_color_optical_frame`。`grasp_offset` 只补偿物体几何，不要拿它掩盖外参误差。

---

## 8. 阶段 G — 维护

出现以下情况重标：

- 相机支架被撞、拆装过；
- 换相机/镜头/分辨率（内参变）；
- 机器人重新做运动学参数同步（`flexiv_calibration`）；
- 重复标定 3 次结果离散 >5 mm → 先查机械结构。

---

## 9. 排错

| 现象 | 处理 |
| --- | --- |
| spawner 卡在 `waiting for /controller_manager` | 机器人不在 Auto (Remote)；看 launch 日志里 `FlexivHardwareInterface` 的 FATAL |
| `tf base <- flange` FAIL | 机器人栈没起或 `robot_type` 给错 |
| `camera image` 0 msgs | 相机没起/话题名不同；看 doctor 打印的 publisher 数量 |
| `corner spacing` FAIL | 太远，移到 0.30–0.35 m |
| `no usable target in view` | 板出画面/反光/太暗 |
| `too close to an existing pose` | 姿态没变够，转 ≥10° |
| `consistency` > 0.5° | 采时机械臂还在动/板动了，重采 |
| `axis spread` < 20° | 只滚转没倾斜，补 x/y 倾斜 |
| `all solvers failed` | 数据退化（纯平移/纯滚转） |

---

## 10. 一分钟速查

```bash
# 启动
ros2 launch flexiv_bringup rizon_moveit.launch.py robot_sn:=Rizon4s-064230 robot_type:=Rizon4s load_gripper:=false
ros2 launch realsense2_camera rs_launch.py rgb_camera.color_profile:=1280x720x30
ros2 run flexiv_handeye_calib calibrate --ros-args -p robot_sn:=Rizon4s-064230

# 自检 → 采样 → 求解
ros2 run flexiv_handeye_calib doctor --ros-args -p robot_sn:=Rizon4s-064230 -p timeout_s:=3.0
ros2 service call /handeye_calibrator/capture std_srvs/srv/Trigger
ros2 service call /handeye_calibrator/solve   std_srvs/srv/Trigger

# 验证
ros2 launch flexiv_handeye_calib validate.launch.py robot_sn:=Rizon4s-064230
ros2 service call /handeye_validator/record std_srvs/srv/Trigger
ros2 service call /handeye_validator/report std_srvs/srv/Trigger

# 自动摆位（可选）
ros2 run flexiv_handeye_calib auto_move --ros-args -p robot_sn:=Rizon4s-064230 -p max_poses:=2
ros2 service call /auto_move/run std_srvs/srv/Trigger
```

时间预算：启动 10 min、自检 2 min、采样 15 min（手动）/ 5 min（自动）、求解 2 min、
验证 20 min、点触 20 min。
