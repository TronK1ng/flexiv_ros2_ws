# Flexiv Rizon 4s 手眼标定（eye-in-hand）— 方案、实施与精度验证

| 项 | 内容 |
| --- | --- |
| 适用平台 | Rizon 4s + 腕部相机（Intel RealSense D415 为默认假设）+ ROS 2 Humble |
| 标定类型 | eye-in-hand（相机固定在法兰/腕部，标靶固定在世界中） |
| 交付代码 | `src/flexiv_handeye_calib/`（自包含，无 `cv_bridge`） |
| 关联文档 | `docs/grasp_demo_requirements.md`（抓取 demo，M2 视觉链路验证） |

---

## 1. 结论先行

**有开源方案，但都不适合直接嵌进这套工程**，所以本项目采用"开源算法 + 自写工程壳"：

- 算法完全用 OpenCV `calibrateHandEye`（Tsai / Park / Horaud / Andreff / Daniilidis）+ 一个 numpy 非线性精化，
  没有重新发明数学；
- 工程壳解决三个开源方案都不覆盖的问题：① 直接吃本工程已有的 25h9 AprilTag 链路；② 不引入 `cv_bridge`
  （避免 pip OpenCV 4.13 与 apt OpenCV 4.5.4 同进程冲突）；③ 输出可直接给抓取 demo 用的 YAML，并内置
  留一交叉验证 / 多解法一致性 / 独立位姿复现等量化验证。

如果你更想要现成 GUI、且能接受再装几个包，`easy_handeye2` 是成熟选择（见第 2 节）。

---

## 2. 开源方案调研

| 方案 | 状态 | 依赖 / 工作量 | 标靶 | 适配本工程的问题 |
| --- | --- | --- | --- | --- |
| [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2)（ROS 2 移植版 0.5.0） | 成熟，rqt GUI，支持 eye-in-hand / eye-on-base | 需要 `transforms3d`（pip）、`rqt_gui`、Qt；需要有节点把 `camera→tag` 发布成 **tf** | 任意，需自己提供检测节点（`aruco_ros` / `ros2_aruco`） | 本身不含 AprilTag 检测；采样靠手动；自动运动 GUI 只在 Iron 上可用；输出格式与本工程 demo 不同 |
| [moveit_calibration](https://github.com/moveit/moveit_calibration/tree/ros2)（`ros2` 分支） | MoveIt 官方，RViz 插件，ArUco / ChArUco | 需源码编译 `moveit_calibration_gui` + `moveit_calibration_plugins`，依赖 `cv_bridge`、`moveit_ros_visualization` | ArUco / ChArUco 板 | 与本机 MoveIt 2.5.10 的 API 兼容性有已知问题（[moveit2_tutorials#831](https://github.com/moveit/moveit2_tutorials/issues/831)）；`cv_bridge` 会把 apt OpenCV 拉进进程 |
| [moveit2_handeye_calib](https://github.com/cheng9911/moveit2_handeye_calib) | 打包好的示例工程（Franka + moveit2_calibration） | 重，含整套 Franka 驱动 | ArUco / ChArUco | 标定部分就是 `moveit_calibration`，Flexiv 侧接口要自己改 |
| Flexiv 官方 `flexiv_calibration`（本工作区已有） | 可用 | 无需额外依赖 | — | **不是手眼标定**：它把机器人实际运动学参数同步进 URDF（需 RDK 专业 license），是手眼标定的前置优化项 |
| 其它（`industrial_calibration`、`kalibr` 等） | ROS 1 为主 / 面向相机-IMU | 重 | — | 不适用 |

### 2.1 easy_handeye2 怎么用（如果你选它）

```bash
pip install transforms3d
git clone https://github.com/marcoesposito1988/easy_handeye2.git ~/handeye2_ws/src/easy_handeye2
cd ~/handeye2_ws && rosdep install -iyr --from-paths src && colcon build
```

它要求 tf 里同时存在 4 个帧：`robot_base_frame`、`robot_effector_frame`、
`tracking_base_frame`（相机光学系）、`tracking_marker_frame`（标靶）。最后两个需要
一个 ArUco 检测节点发布（`ros2_aruco` 或 `aruco_ros`），否则它采不到样本。
结果存在 `~/.ros/easy_handeye2/<name>.calib`，用 `publish.launch.py` 发布 tf。

**这套组合的短板**正好是本工程需要的部分：样本质量没有任何量化指标，`rqt_evaluator`
只做一致性检查，出问题很难定位。所以下面用自写包，但算法与 easy_handeye2 完全同类
（都是 `cv2.calibrateHandEye`），结果可以互相校验。

### 2.2 为什么自写

1. 复用 `grasp_perception.py` 的 25h9 检测约定，标定与运行时用同一套 tag 语义；
2. 不引入 `cv_bridge`（`docs/grasp_demo_requirements.md` 第 8.1 节已论证过这个风险）；
3. 直接输出 `xyz + rpy + 四元数 + 4×4`，抓取 demo 的 static tf 一行就能接；
4. 内置验证指标（第 6 节），而不是"跑完看着差不多就行"。

实现约 2100 行（含注释与 docstring）+ 340 行测试，只依赖 numpy / OpenCV / PyYAML / rclpy。

---

## 3. 原理与坐标约定

### 3.1 标定方程

eye-in-hand 下，标靶固定、相机随法兰运动，对第 i 个位姿：

```
T_base_flange(i) · T_flange_camera · T_camera_tag(i) = T_base_tag   （常量）
        A_i        ·        X        ·        B_i       =     Y
```

- `A_i`：机器人正运动学（tf `base <- flange`），本包直接查 tf，不自己算 FK；
- `B_i`：视觉位姿（PnP 得到 `T_camera_tag`）；
- `X`：待求外参 `T_flange_camera`；
- `Y`：标靶在 base 下的位姿，是" nuisance 参数"，最后丢弃。

求解：先用相对运动 `A_ij = A_j⁻¹A_i`、`B_ij = B_j B_i⁻¹` 化成 `A_ij X = X B_ij`，
旋转用 Kabsch/极分解闭式解，平移用线性最小二乘，再对 `(X, Y)` 做 12 维 LM 精化。
旋转和平移都可解的条件是**存在足够多不同轴的旋转**；纯平移无解（本包会直接报错）。

### 3.2 帧的约定（容易错的地方）

| 量 | 本包取值 | 说明 |
| --- | --- | --- |
| `base_frame` | `{robot_sn}_base_link` | 缺省由 `robot_sn` 推导 |
| `flange_frame` | `{robot_sn}_flange` | URDF 里的法兰坐标系 |
| `camera_frame` | `camera_link` | **标定结果的子帧**，也是 demo 里 static tf 的子帧 |
| 图像帧 | `camera_color_optical_frame` | 检测在光学系里做，再用 tf 转到 `camera_frame` |
| `tag_size` | `0.026` | **黑色方块外沿边长，含黑边，不含白边/quiet zone** |
| `rpy` | 弧度 | 固定轴 XYZ（`R = Rz·Ry·Rx`），与 `tf2` 的 `rpy` 一致 |

> 标靶自身的坐标轴朝向（x 向右 / y 向上 / z 指向相机）不影响标定结果，只要每帧一致；
> 本包与 `grasp_perception` 采用同一约定，便于对照调试。

### 3.3 位姿选取原则

精度主要取决于**旋转多样性**，不是平移量：

- 建议 15–20 个位姿；绕法兰 x、y 各 ±30°～45°，绕光轴（z）±30°～45°；
- 相邻位姿旋转差 ≥ 10°，否则对 `X` 的约束几乎不变；
- 标靶尽量在画面中央，距离 0.30–0.40 m（26 mm tag 的推荐区间）；
- 不要只在同一姿态附近平移——那是最常见的"看起来采了很多帧、其实信息量为零"。

包内报告会给出 `max/mean relative rotation`，低于 25° 会直接警告。

---

## 4. 硬件与准备

### 4.1 标靶（精度第一来源）

支持两类标靶，用 `target_type` 切换：

| `target_type` | 参数 | 说明 |
| --- | --- | --- |
| `apriltag` | `tag_family` / `tag_size` / `tag_id` | 25h9、36h11 等 AprilTag/ArUco |
| `chessboard`（默认） | `chessboard_cols` / `chessboard_rows` / `square_size` / `chessboard_auto` | 普通棋盘格，**参数填内角点数，不是方格数** |

#### 4.1.1 AprilTag

- 26 mm 的 25h9 在 0.35 m 处只有约 68 px 宽（1280×720、fx≈920），单帧距离噪声约 1 mm；
  **如果条件允许，建议打印 50–80 mm 的 36h11 tag**，同样距离下像素数翻 2–3 倍，精度直接翻倍。
- 用卡尺量**黑色方块外沿**（不是白边）填入 `tag_size`。尺寸差 1% → 距离差 1%（0.35 m 上 3.5 mm）。
- 打印后贴到刚性平板上（亚克力/铝板），四角压平；纸张翘曲 1 mm 就会带来同量级误差。
- quiet zone 至少留 2 个模块宽的白边；表面用亚光材料，避免反光。
- **标定全过程标靶不能移动**，包括被线缆碰到。建议双面胶 + 胶带双重固定。

#### 4.1.2 棋盘格（如 DFOPTIX Q12-100-6）

Q12-100-6 是**玻璃基光刻棋盘格**，属于"方格标定板"，与本包最初的 AprilTag 链路不是一类，需要设置
`target_type:=chessboard`。

实测规格（本工作区使用的那块）：

- **12 格 × 6 mm = 72 mm** 为长边，短边 **9 格 × 6 mm = 54 mm**；
- 内角点 **8 × 11**（`chessboard_cols:=8`、`chessboard_rows:=11`），图案 54 × 72 mm；
- `square_size: 0.006`。

> 型号里的数字不一定等于边长，务必实测：量 10 格总宽除以 10。尺寸错 2 倍会让标定出来的相机位置
> 也差 2 倍，而残差依然很小——这是最难发现的一类错误。

由于方格只有 6 mm，工作距离要比大板近：

| 距离 | 每格像素 | 图案尺寸(px) |
| --- | --- | --- |
| 0.25 m | 22 | 261 × 196 |
| 0.30 m | 18 | 218 × 163 |
| 0.35 m | 15.5 | 187 × 140 |
| 0.40 m | 13.6 | 163 × 122 |

建议 **0.30–0.35 m**（每格 ≥15 px）。注意夹爪长度：太近时倾斜容易碰到桌面。

使用注意：

- 玻璃基板是**镜面反射**：环光/同轴光容易在板上打出高光，导致大片角点丢失。用侧向漫射光，
  或让相机稍微偏一点角度拍。
- 板子必须**完全静止且与桌面贴合**，玻璃板薄，边缘翘起会引入系统性倾斜误差。
- 板是**长方形**，`patternSize` 必须锁定在 `(8, 11)`、`chessboard_auto:=false`。若让自动扫描在
  `(8,11)` 和 `(11,8)` 之间跳变，两种标注对应不同的标靶坐标系，会直接破坏标定。
- 长方形板只有 **180° 对称歧义**，本包用 `normalize_symmetric_board: true` 自动纠正；
  `solve` 输出里的 `symmetry: realigned N samples` 就是纠正数量。

### 4.2 相机

- 1280×720 彩色流；固定曝光/增益/白平衡（`rqt_reconfigure` 里关掉自动曝光），避免逐帧变化。
- 用驱动发布的 `camera_info` 内参；若对精度要求高，先用棋盘格重标内参（内参误差会 1:1 进入外参）。
- D415 的彩色相机与深度相机有固定基线，本流程只用彩色图 + PnP，不需要深度。
- 相机与法兰必须刚性连接：支架不能有弹性，线缆要留松弛量，避免拖拽改变外参。

### 4.3 机械臂

- **模式必须是 Auto (Remote)**。Flexiv 的 RDK 驱动在所有其它模式下都会在 activate 阶段直接失败，
  典型表现是 `ros2_control_node` 退出、三个 spawner 卡在
  `waiting for service /controller_manager/list_controllers`。日志里会有：

  ```
  [WARN] [FlexivHardwareInterface]: Robot is in regular Auto mode. Switch to Auto (Remote) in Flexiv Elements.
  [FATAL] [FlexivHardwareInterface]: Cannot start: Robot is in regular Auto mode...
  [ERROR] [ros2_control_node-1]: process has died [exit code -6]
  ```

  切换方法（[Flexiv 官方说明](https://www.flexiv.com/software/rdk/manual/v2.x/enter_and_exit_remote_mode.html)）：
  1. 把 motion bar 上的 **Auto/Manual 拨动开关**拨到**上方**（Auto）；
  2. Flexiv Elements 里弹出提示，选 **AUTO (REMOTE)**；
  3. 若本来就是 Auto 模式，先把开关拨到下方（Manual）再拨回上方，提示才会重新出现；
  4. 切完后 Elements 只能读数据、不能下发指令，这是正常的。

  释放急停、使能，再重启 launch。
- 先跑一次 `flexiv_calibration`（运动学参数同步，需要 RDK 专业 license），能减小 FK 误差；
  没有 license 也能标定，只是绝对精度受限于机器人出厂精度。
- 速度 10% 起步，标定过程有人守急停。
- 用 `use_fake_hardware:=true` 可以先把软件链路跑通（不会真的动）。

---

## 5. 操作流程

### 5.1 启动

```bash
# 方式 A：一条命令拉起 MoveIt + RealSense + 标定节点（AprilTag）
ros2 launch flexiv_handeye_calib calibrate.launch.py \
    robot_sn:=Rizon4s-123456 robot_type:=Rizon4s \
    tag_size:=0.026 auto_capture:=true

# 棋盘格（DFOPTIX Q12-100-6：6x6 方格 -> 5x5 内角点，12 mm）
ros2 launch flexiv_handeye_calib calibrate.launch.py \
    robot_sn:=Rizon4s-123456 robot_type:=Rizon4s \
    target_type:=chessboard chessboard_cols:=8 chessboard_rows:=11 \
    square_size:=0.006 chessboard_auto:=true auto_capture:=true

# 方式 B：机器人栈和相机已经在跑
ros2 launch flexiv_handeye_calib calibrate.launch.py \
    robot_sn:=Rizon4s-123456 launch_bringup:=false start_camera:=false
```

### 5.2 自检

```bash
# AprilTag
ros2 run flexiv_handeye_calib doctor --ros-args -p robot_sn:=Rizon4s-123456

# 棋盘格：让 doctor 直接告诉你它认出了几乘几的内角点
ros2 run flexiv_handeye_calib doctor --ros-args -p robot_sn:=Rizon4s-123456 \
    -p target_type:=chessboard -p square_size:=0.006 -p chessboard_auto:=true
```

逐项输出 `[PASS]/[FAIL]`：图像、内参、`base<-flange`、`camera<-optical`、图像时间戳、
tag 是否可见、tag 像素宽度。任何一项 FAIL 都不要开始采样。

### 5.3 采样

两种方式，效果一样：

- **手动**：在 RViz 的 MotionPlanning 面板拖末端到不同姿态，或示教器/手动拖动；
  每到一个位姿停稳，执行
  `ros2 service call /handeye_calibrator/capture std_srvs/srv/Trigger`。
- **自动**（`auto_capture:=true`）：节点每 0.2 s 检查一次，机械臂停稳（速度 < `auto_max_speed`）
  且相对上一个样本移动超过 `auto_min_translation` / `auto_min_rotation_deg` 时自动采样。
  操作者只需要"摆姿态 → 停一下 → 再摆"。

采样成功的日志会给出 `id / reproj / area / range`，并立刻落盘到
`~/.ros/flexiv_handeye/samples.json`（断电/崩溃不丢）。

**怎么知道转了多少度**：RViz 的旋转圆环没有读数。开一个终端跑

```bash
ros2 run flexiv_handeye_calib pose_delta --ros-args -p robot_sn:=Rizon4s-123456
```

它每 0.5 s 比较一次当前位姿与参考位姿，位姿变化超过 0.5°/1 mm 就打印：

```
d_rot  21.3 deg (axis +0.98 +0.05 +0.12)  d_trans   3.2 mm  |  total  21.3 deg  |  pos [0.412 -0.131 0.508]
```

`d_rot` 是相对上一个 `~/mark` 的转角，`total` 是相对第一个位姿的累计转角（用来判断多样性够不够）。
换姿态前调一次 `ros2 service call /pose_delta/mark std_srvs/srv/Trigger` 归零。

纯滚转也可以直接在 MotionPlanning 的 **Joints** 标签里改 J7 的数值，比拖拽准。

建议分布：

```
俯仰：+35°, +20°, 0°, -20°, -35°
偏航：+35°, +20°, 0°, -20°, -35°
滚转：+45°, +20°, 0°, -20°, -45°
距离：0.30 / 0.35 / 0.40 m 各若干
```

### 5.3b 自动摆位（`auto_move`）

让机械臂自己走完 14 个姿态，人只在开场摆好观察位、结束时看报告。

**它不走 RDK 直连**：机器人的运动控制权在 `ros2_control` 手里（同一 SN 的 RDK 连接，1 kHz 下发关节指令），
另起一个 RDK 客户端发 `MoveCart` 会互相覆盖。`auto_move` 用的是同一条合法通道：

- 逆解：`/compute_ik`（MoveIt）
- 执行：`/rizon_arm_controller/follow_joint_trajectory`（JTC）

**怎么保证标靶不出画面**：相机不在法兰轴线上，倾斜会使标靶在画面里大幅平移。启动后它先做两次
2 cm 的平移探针，测出「法兰平移 → 像素位移」的 2×2 雅可比；之后每个姿态转完都用它把标靶拉回原位。
大角度分两段转，每段都先回中，所以标靶不会在中途跑出视野。

**启动顺序**（终端 T1/T2 照旧）：

```bash
# T3：标定节点，关掉引导提示（避免和 auto_move 的进度互相干扰）
ros2 run flexiv_handeye_calib calibrate --ros-args \
    -p robot_sn:=Rizon4s-064230 -p target_type:=chessboard -p guided:=false

# T4：自动摆位节点（启动后不会动）
ros2 run flexiv_handeye_calib auto_move --ros-args -p robot_sn:=Rizon4s-064230

# 先看计划，不动
ros2 run flexiv_handeye_calib auto_move --ros-args -p robot_sn:=Rizon4s-064230 -p dry_run:=true
```

**第一次试跑**：只走两个姿态，确认方向对：

```bash
ros2 run flexiv_handeye_calib auto_move --ros-args -p robot_sn:=Rizon4s-064230 -p max_poses:=2
ros2 service call /auto_move/run std_srvs/srv/Trigger
```

确认没问题后去掉 `max_poses` 跑完整轮（14 个姿态，约 3–5 分钟）。中途：

```bash
ros2 service call /auto_move/abort std_srvs/srv/Trigger   # 取消并停住
```

**安全约束**：

- 不调 `~/run` 不会动；调用后手不要离开急停；
- 每个目标位姿都要 IK 成功、且离起始位姿不超过 `max_translation`（默认 15 cm）才执行；
- 标靶一旦检测不到或贴到画面边缘，立即停止并报 "move the arm back by hand"；
- 速度由轨迹时长控制（默认单步 3 s，实际会更慢）。

**已知限制**：本功能未在真机上验证过。先用 `max_poses:=2` 试跑，确认转向、回中方向都符合预期。

### 5.4 求解

```bash
ros2 service call /handeye_calibrator/solve std_srvs/srv/Trigger
```

返回一段报告（同时写日志、写 YAML、发布 `flange -> camera_link` 的 static tf）：

```
frames      : Rizon4s-123456_flange -> camera_link
method      : PARK
samples     : 16 used, 0 rejected
residual    : 0.75 mm rms / 1.36 mm max, 0.164 deg rms / 0.321 deg max
leave-one-out: 1.06 mm rms, 0.173 deg rms
rotation diversity: max 38.1 deg, mean 17.2 deg
method spread : 0.39 mm, 0.161 deg
result      : xyz=[0.0435, -0.0189, 0.0748] m  rpy=[-0.12, -34.53, 90.19] deg
verdict     : GOOD
```

### 5.5 离线复算 / 对比解法

```bash
ros2 run flexiv_handeye_calib handeye_solve \
    --samples ~/.ros/flexiv_handeye/samples.json \
    --method DANIILIDIS --json /tmp/metrics.json
```

换 5 种解法各跑一次，`result` 的离散程度本身就是数据质量的度量（见 6.4）。

### 5.6 执行器输出的 YAML

```yaml
handeye_calibration:
  type: eye_in_hand
  parent_frame: Rizon4s-123456_flange
  child_frame: camera_link
  xyz: [0.0435, -0.0189, 0.0748]        # m
  rpy: [-0.0021, -0.6027, 1.5742]       # rad，tf2 约定
  quaternion: [0.2095, -0.2102, 0.6762, 0.6743]
  matrix: [[...], ...]                  # 4x4
  quality: {...}
```

同时包含 `xyz/rpy`、四元数与 4×4 矩阵，三种用法都能直接取。

---

## 6. 如何验证标定的准确性（核心）

标定的"精度"有两个不同的问题：**拟合得好不好**（模型一致性）和**准不准**（绝对真值）。
前者用残差就能看，后者必须引入独立信息。下面 7 层从便宜到贵，建议至少做到第 5 层。

### 6.0 先明确一个陷阱

`A X B = Y` 的拟合残差**无法发现标靶尺寸错误**：如果 `tag_size` 填大 10%，视觉给出的距离会
系统性偏小 10%，但这个偏差会被 `X` 的平移和 `Y` 一起吸收，残差仍然接近 0。同理，相机内参
的整体缩放误差也会被吸收。所以**残差小 ≠ 准**，必须有独立于"同一套视觉测量"的检查（6.5/6.6）。

### 6.1 第 1 层：单帧观测质量（采样时看）

| 指标 | 好 | 处理 |
| --- | --- | --- |
| 重投影误差 | < 1 px | > 3 px 直接丢弃（`max_reprojection_error_px`） |
| tag 像素宽度 | ≥ 50 px | < 30 px 时距离噪声 > 2 mm，换近一点或换大 tag |
| 距离 | 0.30–0.40 m | 超出范围精度下降 |
| 图像年龄 | < 0.2 s | 过大说明图像停了/时间戳不对 |

这是必要条件，不是充分条件。

### 6.2 第 2 层：拟合残差（solve 自动给出）

`residual: X mm rms / Y mm max, A deg rms / B deg max` 表示每个位姿下
`A_i X B_i` 与公共 `Y` 的偏差。

| 等级 | 平移 RMS | 旋转 RMS |
| --- | --- | --- |
| 优秀 | < 1 mm | < 0.2° |
| 可接受 | < 3 mm | < 0.5° |
| 需排查 | > 5 mm | > 1° |

残差大通常是：tag 尺寸错、内参错、采样时机械臂还在动、时间戳不同步、tag 不平、图像模糊。

### 6.3 第 3 层：留一交叉验证（LOO，最关键的一个数）

对每个样本 i，用其余 N−1 个样本重新求解，再预测第 i 个位姿下的标靶位置，与全量解的 `Y` 比较，
得到 `leave-one-out` RMS。

- **LOO ≈ 拟合残差**：模型泛化正常，拟合残差可信；
- **LOO ≫ 拟合残差**（比如 2 倍以上）：位姿太集中/退化，或者有过拟合，需要补姿态；
- LOO 是"换一个新位姿还能不能对上"的直接估计，比拟合残差更接近真实精度。

合成数据实验（16 个位姿，观测噪声 0.5 mm / 0.1°）：拟合残差 0.75 mm，LOO 1.06 mm，
而 `X` 与真值的实际误差是 1.0 mm——三者同一量级，说明这个指标是可信的。

### 6.4 第 4 层：多解法一致性

同一个数据集，用 Tsai / Park / Horaud / Andreff / Daniilidis 分别求解，比较各自 `X` 的差异
（报告的 `method spread`）。

| 离散度 | 判断 |
| --- | --- |
| < 1 mm / 0.2° | 数据条件好，结果可信 |
| 1–3 mm / 0.2–0.6° | 尚可，以 LOO 为准 |
| > 5 mm / > 1° | 数据退化（旋转不足/噪声大），补姿态 |

不同解法对噪声的敏感方向不同，它们一致说明"这个解不是某个算法的偶然产物"。

### 6.5 第 5 层：独立位姿复现（validator，端到端）

保持标靶不动，把机械臂摆到**没有参与标定的新位姿**（至少 8 个，覆盖不同方向），
用已发布的 tf 链（`base→flange→camera→tag`）反算标靶在 base 下的位姿。

```bash
ros2 launch flexiv_handeye_calib validate.launch.py robot_sn:=Rizon4s-123456
# 每到一个新位姿：
ros2 service call /handeye_validator/record std_srvs/srv/Trigger
# 全部录完：
ros2 service call /handeye_validator/report std_srvs/srv/Trigger
```

报告给出标靶位姿的均值、逐轴 std、RMS/最大偏差、旋转 RMS：

```
validation samples : 10
fixed target (base): [0.5123, 0.0442, 0.0217] m
repeatability      : 1.24 mm rms, 2.1 mm max, 0.28 deg rms
per axis std       : x 1.10 y 0.74 z 0.83 mm
verdict            : GOOD
```

| 等级 | RMS | 最大 |
| --- | --- | --- |
| 优秀 | < 1.5 mm | < 3 mm |
| 可接受 | < 3 mm | < 5 mm |
| 需排查 | > 5 mm | > 8 mm |

注意：这一层验证的是"整条链在部署状态下是否自洽"，包括 tf 是否真的按 YAML 发布。
它也仍然抓不到 6.0 说的系统性尺度偏差（所有位姿同向偏），所以还要做 6.6。

### 6.6 第 6 层：物理金标准（探针 / 抓取）

这是唯一能发现系统性尺度/零位偏差的检查，建议至少做一次。

**点触法**：

1. 在桌面上贴一个平放的 tag（或画十字），用相机测出它在 base 下的位置 `p_tag`；
2. 用示教/手动，把 TCP（`{sn}_grav_tcp`，或夹一个尖头工具并更新 TCP）移动到该点正上方，
   缓慢下降直到刚好接触，记录此时 TCP 在 base 下的位置 `p_tcp`；
3. 误差 = `‖p_tag − p_tcp‖`（z 方向扣掉 tag 纸厚/工具尖长，或只比较 xy）；
4. 在桌面上取 8–10 个不同位置重复（覆盖工作空间四角与中心），统计 mean / std / max。

**判据**：mean < 3 mm，max < 5 mm 为合格；< 2 mm 为优秀。
如果 mean 明显偏大但 std 很小 → 系统性偏差（tag 尺寸/内参/TCP 定义）；std 大 → 随机误差。

**抓取法**（最贴近实际用途）：用 20 mm 立方体做 20 次顶抓，统计成功率与抓偏方向：

- 成功率 ≥ 90%，且失败方向随机 → 外参没问题；
- 每次都朝同一方向偏 → 系统性偏差，回到 6.0 检查 tag 尺寸/内参；
- 时好时坏 → 采样噪声或机器人重复性。

### 6.7 第 7 层：重复标定一致性

同一套硬件，重新采样、重新求解 3 次，比较三次 `X`：

| 离散 | 判断 |
| --- | --- |
| < 2 mm / 0.3° | 稳定 |
| 2–5 mm | 采样姿态分布导致的正常波动，增加位姿数 |
| > 5 mm | 硬件有松动（支架/相机/标靶），先查机械结构 |

### 6.8 阈值总表

| 指标 | 优秀 | 可接受 | 需重做 |
| --- | --- | --- | --- |
| 单帧重投影 | < 1 px | < 2 px | > 3 px |
| 拟合残差 RMS | < 1 mm / 0.2° | < 3 mm / 0.5° | > 5 mm / 1° |
| LOO RMS | < 2 mm | < 5 mm | > 8 mm 或 ≫ 拟合残差 |
| 多解法离散 | < 1 mm | < 3 mm | > 5 mm |
| 独立位姿复现 | < 1.5 mm | < 3 mm | > 5 mm |
| 物理点触 | < 2 mm | < 3 mm | > 5 mm |
| 重复标定 | < 2 mm / 0.3° | < 5 mm | > 5 mm |

对抓取 demo（20 mm 物体）来说，外参误差控制在 **3 mm 以内**就足够；26 mm tag 在 0.3–0.4 m
的配置，正常情况下可以做到 2 mm 左右。

---

## 7. 误差预算

| 来源 | 量级 | 对 `X` 的影响 | 控制手段 |
| --- | --- | --- | --- |
| tag 尺寸误差 δs/s | 0.1 mm/26 mm = 0.4% | 距离误差同比 0.4%（0.35 m → 1.4 mm） | 卡尺量黑边，别量白边 |
| 角点噪声 | 0.2 px | 距离 `δz = z²/(f·s)·δp` ≈ 1 mm（z=0.35, f=920, s=26 mm） | 大 tag / 近一点 / 固定曝光 |
| 机器人 FK | 0.2–1 mm | 同量级 | 同步运动学参数（`flexiv_calibration`） |
| 时间戳不同步 | 10 ms × 50 mm/s = 0.5 mm | 同量级 | 停稳再采；用图像时间戳查 tf |
| 相机-法兰刚性 | 0.5–2 mm | 同量级 | 刚性支架、线缆松弛 |
| 样本数 | ~噪声/√N | N=15 比 N=5 好 1.7 倍 | 15–20 个位姿 |

经验公式（单帧距离，便于现场估算）：

```
δz ≈ z² / (f · s) · δp
z=0.35 m, f=920 px, s=0.026 m  →  每 0.2 px 角点噪声 ≈ 1 mm 距离误差
```

---

## 8. 与抓取 demo 对接

### 8.1 发布外参

标定节点在 `solve` 时已经发布 `flange -> camera_link` 静态 tf，并在启动时自动重发 YAML 里的结果。
如果 demo 自己也要发，二选一：

```bash
# 方式 A：让标定节点发（publish_tf:=true，默认）
ros2 run flexiv_handeye_calib calibrate --ros-args \
    -p robot_sn:=Rizon4s-123456 -p publish_tf:=true

# 方式 B：用 static_transform_publisher 发，标定节点 publish_tf:=false
ros2 run tf2_ros static_transform_publisher \
    --x 0.0435 --y -0.0189 --z 0.0748 \
    --qx 0.2095 --qy -0.2102 --qz 0.6762 --qw 0.6743 \
    --frame-id Rizon4s-123456_flange --child-frame-id camera_link
```

**同一个 `(parent, child)` 只能有一个发布者**，否则 tf 会跳变。

### 8.2 与 `grasp_demo.yaml` 的关系

- `grasp_perception` 用 `base_frame <- camera_color_optical_frame` 查 tf，外参正确即可；
- `grasp_offset` 是沿标靶法向的**物体几何补偿**，不是外参补偿，不要用它去掩盖标定误差；
- `docs/grasp_demo_requirements.md` 第 12 节第 1 项"手眼外参待提供"现在可以填本包的输出了。

### 8.3 M2 验收

`docs/grasp_demo_requirements.md` 的 M2 要求"手持标记在 0.3/0.4/0.5 m，`/detect` 输出与卷尺
误差 < 5 mm"。标定完成后，这个检查可以升级为第 6.5 + 6.6 节的两项，阈值收紧到 3 mm。

---

## 9. 排错

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| `no usable tag in view` | 太远/太暗/曝光过长/字典不匹配/面积小于阈值 | 走近、开灯、固定曝光、确认 `tag_family`、调 `min_area_px` |
| 棋盘格检测不到 | 规格填错（方格数当内角点）、反光、只拍到一部分、白边不足 | 数内角点；开 `chessboard_auto`；换侧光；保证整块板在视野内且四周有白边 |
| `detected NxM corners` 与预期不符 | 型号判读错误或只看到子区域 | 用卡尺量方格边长反推：图案宽 = N×边长；必要时手动锁定 `chessboard_cols/rows` |
| `symmetry: realigned` 数量很大 | 方板 90° 歧义，或 `square_size`/内角点数错 | 先确认规格；采样时限制滚转；仍大就在旁边加一个 AprilTag |
| `no camera frame received yet` | 话题名不对/相机没起 | `ros2 topic hz` 确认，`image_topic` 改对 |
| `too close to an existing pose` | 没移动够 | 换姿态，或调小 `dedupe_translation/rotation` |
| `tf ... unavailable` | 帧名错/`robot_state_publisher` 没起/时间戳早于 tf | `ros2 run tf2_tools view_frames`，确认 `{sn}_base_link`、`{sn}_flange` |
| 残差 > 5 mm | tag 尺寸、内参、运动模糊、时间戳 | 逐项按第 7 节排查；先看单帧重投影 |
| `low rotation diversity` | 只平移/只小角度 | 补 ±30–45° 的俯仰/偏航/滚转 |
| LOO 远大于拟合残差 | 位姿集中、过拟合 | 加姿态，尤其不同轴的旋转 |
| `all solvers failed` | 纯平移/数据退化 | 至少要有两个不同轴的旋转 |
| 结果平移明显不符合实际安装 | 单位（mm/m）、tag_size 用了白边、内参 | 卡尺复测，注意 YAML 是米 |
| 标定后整体抓偏一个固定方向 | 系统性偏差 | 做 6.6 点触法，查 tag 尺寸与 TCP 定义 |
| 抓取时好时坏 | 随机噪声/机器人重复性 | 加位姿数、固定曝光、降速 |

---

## 10. 命令速查

```bash
# 自检
ros2 run flexiv_handeye_calib doctor --ros-args -p robot_sn:=Rizon4s-123456

# 启动
ros2 launch flexiv_handeye_calib calibrate.launch.py robot_sn:=Rizon4s-123456 \
    tag_size:=0.026 auto_capture:=true

# 采样 / 求解 / 状态 / 清空
ros2 service call /handeye_calibrator/capture std_srvs/srv/Trigger
ros2 service call /handeye_calibrator/solve   std_srvs/srv/Trigger
ros2 service call /handeye_calibrator/status  std_srvs/srv/Trigger
ros2 service call /handeye_calibrator/reset   std_srvs/srv/Trigger

# 棋盘格（Q12-100-6）
ros2 launch flexiv_handeye_calib calibrate.launch.py robot_sn:=Rizon4s-123456 \
    target_type:=chessboard chessboard_cols:=8 chessboard_rows:=11 \
    square_size:=0.006 auto_capture:=true

# 离线复算 / 换解法
ros2 run flexiv_handeye_calib handeye_solve --samples ~/.ros/flexiv_handeye/samples.json \
    --method DANIILIDIS --json /tmp/m.json

# 独立验证
ros2 launch flexiv_handeye_calib validate.launch.py robot_sn:=Rizon4s-123456
ros2 service call /handeye_validator/record std_srvs/srv/Trigger
ros2 service call /handeye_validator/report std_srvs/srv/Trigger

# 测试（无需硬件）
colcon test --packages-select flexiv_handeye_calib
cd src/flexiv_handeye_calib && PYTHONPATH=. python3 -m pytest test -q
```

---

## 11. 参考文献

1. R. Y. Tsai, R. K. Lenz, *A new technique for fully autonomous and efficient 3D robotics hand/eye calibration*, 1989.
2. F. C. Park, B. J. Martin, *Robot sensor calibration: solving AX=XB on the Euclidean group*, 1994.
3. K. Daniilidis, *Hand-eye calibration using dual quaternions*, 1998.
4. OpenCV `calibrateHandEye` / `solvePnP` 文档。
5. [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2)、
   [moveit_calibration](https://github.com/moveit/moveit_calibration/tree/ros2)、
   [MoveIt 手眼标定教程](https://moveit.picknik.ai/humble/doc/examples/hand_eye_calibration/hand_eye_calibration_tutorial.html)。
