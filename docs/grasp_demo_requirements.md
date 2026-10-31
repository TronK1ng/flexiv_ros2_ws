# Flexiv 抓取 Demo — 需求与设计确认单

| 项 | 内容 |
| --- | --- |
| 版本 | v1.0 |
| 日期 | 2026-10-29 |
| 状态 | M0 已完成；M1 起待真机联调（剩余项见第 12 节） |
| 目标平台 | Rizon 4s（SN `Rizon4s-064230`）+ Flexiv Grav (Flexiv-GN01) + Intel RealSense D415 |
| 代码基线 | 本工作区 `src/flexiv_ros2`（humble-v1 线）+ `src/flexiv_description` + `src/flexiv_rdk` |

> v0.1 → v0.2：补充用户提供的标记规格（25h9 / 26 mm / 六面贴标）、触发方式（用户手动调 action）、
> 真机安全约束；并根据本机实际软件环境调整感知实现方案。
>
> v0.2 → v0.3：确认标记为 AprilTag 25h9；M0（fake hardware 全流程）已跑通并留了冒烟测试脚本；
> 修正 `grasp_offset` 的符号约定（负值下探）与"深度仅作校验"的表述（v1 不使用深度）。
>
> v0.3 → v0.4：明确目标物体为 40×40×40 mm 立方体、20 g；据此收紧夹爪开口（70 mm）、
> 抓取力（10 N）与下探深度（-20 mm，TCP 落在立方体中心高度）。
>
> v0.4 → v0.5：接入示教器读出的放置点 (488.82, -251.60, -50.21) mm / RPY (0, 180, 9.62)°；
> 确认 URDF 中 `world → {sn}_base_link` 为恒等变换，示教器 World 读数可直接当 base 系坐标；
> 观察位改为由放置点自动派生（正上方 0.30 m）。

---

## 1. 目标与范围

一段可在真机上演示的 ROS 2 抓取流程：机械臂移动到观察位 → 腕部相机识别贴有标记的物体 → 规划并执行抓取 →
抬起 → 运到固定放置点 → 松开 → 退回。整个过程由一个 ROS 2 action 触发，用户用
`ros2 action send_goal` 即可跑一次，RViz 中可观察。

**做**：

- 单物体、静止、桌面、从上往下（顶抓）。
- 开环：观察位拍一次照，算一次位姿，执行到底，不做视觉伺服。
- 抓取成功与否以夹爪状态（宽度 / 力）判定，不做二次视觉确认。
- 一次一个物体；失败即返回错误码，不自动重试。

**不做**：见第 13 节。

---

## 2. 已核实的运行环境

| 项 | 现状 |
| --- | --- |
| OS / ROS | Ubuntu 22.04.5 + ROS 2 Humble ✅ |
| 工作区 | 已编译完成，`install/` 下 10 个包齐全 ✅ |
| RDK | C++ 版已装（`flexiv_install/lib/libflexiv_rdk.a`）✅；**Python 绑定未安装**（不影响本方案，任务层用 C++） |
| MoveIt | 已装 MoveIt 2.5.10 ✅；**`moveit_py` / `pymoveit2` 均未安装** → 任务层用 C++ `MoveGroupInterface` |
| OpenCV | 系统 Python 可见 OpenCV 4.13（用户级 pip，`~/.local`），含 `cv2.aruco.DICT_APRILTAG_25H9` ✅ |
| 相机 | `realsense2_camera` 已装 ✅；当前未插 D415（无 `/dev/video*`） |
| 网络 | `eno1` 当前 DOWN / NO-CARRIER（机器人未接）；`wlp4s0` 192.168.3.88 |
| 机器 | 16 核 / 15 GB 内存 |

**结论**：软件栈齐备，不需要新增 apt/pip 依赖即可实现 AprilTag 25h9 检测（OpenCV 4.13 自带该字典）。

---

## 3. 场景与假设

1. 物体是 **40×40×40 mm 的立方体、20 g**，静止放在桌面上，**六面都贴有标记**（26 mm，family 25h9）。
2. 相机装在腕部（eye-in-hand），与 flange 刚性连接；手眼外参由用户提供。
3. 抓取姿态为**顶抓**：TCP 的 z 轴竖直向下，与物体顶面标记的法向相反。
4. 观察位（拍照位姿）需要示教，且必须让物体完整落入相机视野。
5. 抓取区与放置点都在 Rizon 4s 的可达工作空间内，且路径无遮挡。
6. 演示时有人守急停；速度按 10% 起步。

> 假设 1/3/5 若与实际不符（例如物体是长条、只能侧抓），抓取姿态生成逻辑需要改，见第 12 节待确认项。

---

## 4. 系统架构

```
                    ┌──────────────────────┐
                    │  realsense2_camera   │  (复用，D415)
                    │  color / depth / info│
                    └──────────┬───────────┘
                               │ image, aligned_depth, camera_info
                               ▼
   static TF              ┌──────────────────┐
   flange→camera_link ───►│ grasp_perception │  (新，Python)
   (用户外参)              │  AprilTag 25h9   │
                          └────────┬─────────┘
                                   │ service /grasp_demo/detect
                                   │ → GraspTarget[] (base 系)
                                   ▼
   ┌───────────────┐        ┌──────────────────┐        ┌──────────────────┐
   │  move_group   │◄──────►│   grasp_task     │◄──────►│ flexiv_gripper_  │
   │  (复用,MoveIt)│ MoveIt │  (新，C++ 状态机)│ action │ node (复用)       │
   └───────┬───────┘        └──────────────────┘        └────────┬─────────┘
           │                                                   │
           ▼                                                   ▼
   rizon_arm_controller                        /flexiv_gripper_node/{move,grasp,stop}
   (joint_trajectory_controller)                          Grav 夹爪
```

| 组件 | 语言 | 新增/复用 | 职责 |
| --- | --- | --- | --- |
| `grasp_perception` | Python | 新增 | 订阅彩图/深度/内参，检测 25h9 标记，反投影出 6DoF 位姿，转 base 系，提供按需检测服务 |
| `grasp_task` | C++ | 新增 | 状态机：观察 → 检测 → 规划 → 抓取 → 抬起 → 放置 → 松开 |
| `move_group` | C++ | 复用 | 规划与执行（`rizon_moveit.launch.py` 启动） |
| `flexiv_gripper_node` | C++ | 复用 | `/move`、`/grasp`、`/stop` |
| `realsense2_camera` | C++ | 复用 | 图像流与内参 |
| `flexiv_hardware` / JTC | C++ | 复用 | 真机接口与轨迹执行 |

---

## 5. 接口定义

### 5.1 Action：`flexiv_grasp_demo/action/PickPlace.action`

```
# ---- Goal ----
string object_id                    # 目标标记 ID（十进制字符串）；空 = 视野内面积最大的标记
geometry_msgs/PoseStamped grasp_pose   # 可选，直接给 base 系抓取位姿；position 全 0 表示用视觉结果
geometry_msgs/PoseStamped place_pose   # 可选，空则用 YAML 中的默认放置点
float64 grasp_force                 # N；0 = 用 YAML 默认值
float64 approach_height             # m；0 = 用 YAML 默认值
---
# ---- Result ----
bool success
string message                      # 失败原因；成功时为 "ok"
geometry_msgs/PoseStamped grasp_pose_used   # 实际使用（或测得）的抓取位姿，便于复盘
float64 measured_width              # 闭合后夹爪宽度，m
---
# ---- Feedback ----
string stage                        # OBSERVE/DETECT/APPROACH/GRASP/LIFT/PLACE/RELEASE/RETREAT
float64 progress                    # 0.0 ~ 1.0
string detail                       # 人可读的补充信息
```

调用示例（用户即调用方）：

```bash
ros2 action send_goal /grasp_demo/pick_place flexiv_grasp_demo/action/PickPlace \
  "{object_id: '7', grasp_force: 20.0, approach_height: 0.10}" --feedback
```

### 5.2 Service：`flexiv_grasp_demo/srv/Detect.srv`

```
string object_id        # 空 = 返回全部检测结果
---
GraspTarget[] targets
bool image_stale        # 最新帧时间戳超过阈值（默认 0.2 s）时为 true
string message
```

### 5.3 Message：`flexiv_grasp_demo/msg/GraspTarget.msg`

```
uint16 id                     # 标记 ID
geometry_msgs/PoseStamped pose    # 标记中心在 base 系下的位姿，z 轴指向标记外法向
geometry_msgs/Vector3 size        # 标记边长（m），供校验
float64 reprojection_error        # px，质量指标
float64 area                      # px²，用于"选哪个面"的判据
```

### 5.4 复用接口（不改）

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/flexiv_gripper_node/move` | `flexiv_msgs/action/Move` | 开合到指定宽度 |
| `/flexiv_gripper_node/grasp` | `flexiv_msgs/action/Grasp` | 力控抓取 |
| `/flexiv_gripper_node/stop` | `std_srvs/srv/Trigger` | 急停夹爪 |
| `/{sn}_arm_controller/follow_joint_trajectory` | 标准 JTC action | 由 MoveIt 间接使用 |

---

## 6. 坐标与 TF

已标定的外参（`Rizon4s-064230_flange -> camera_link`，eye-in-hand）：

| 项 | 值 |
| --- | --- |
| xyz | `[-0.081268, -0.009562, 0.022950]` m |
| quaternion | `[-0.004745, -0.511938, 0.003057, 0.859004]` |
| rpy | `[-0.023708, -1.074777, 0.021245]` rad |
| 质量 | 24 组样本，Park 法，平移 RMS 1.63 mm / 旋转 RMS 0.63°，验证重复性 4.16 mm rms |
| 发布 | `handeye_tf_publisher.py` → `/tf_static` |

**相机倾角**：`camera_link` 的 x 轴（光轴）与 flange z 轴夹角 **28.4°**。工具竖直朝下时，
光轴指向 base 的 `(-0.47, -0.07, -0.88)`，视野中心落在 TCP 竖直下方约 5–6 cm 处（朝 -x）。
0.30 m 处 D415 半视场约 0.21 m，因此**观察位保持工具竖直朝下即可**，不需要为相机倾角专门调整姿态。


| frame | 来源 | 说明 |
| --- | --- | --- |
| `{sn}_base_link` | `robot_state_publisher` | 规划与位姿计算的基准系 |
| `{sn}_flange` | `robot_state_publisher` | 手眼外参的父系 |
| `camera_link` | static（demo 发布） | 由用户外参 `T_flange_camera` 决定 |
| `camera_color_optical_frame` | `realsense2_camera` | OpenCV 光学系（z 向前、x 向右、y 向下） |
| `{sn}_grav_tcp` | URDF | 夹爪 TCP（张开手指） |

- 外参以 YAML 形式给出（`xyz` + `rpy`，欧拉顺序建议 RPY/固定轴 XYZ），launch 时由
  `tf2_ros static_transform_publisher` 发布 `{sn}_flange → camera_link`。
- 感知节点内部：`camera_color_optical_frame` → `{sn}_base_link` 的变换通过 `tf2_ros` 查，**不使用手写矩阵**。
- `camera_link → camera_color_optical_frame` 由驱动发布，不重复建。

---

## 7. 参数（`config/grasp_demo.yaml`）

```yaml
grasp_task:
  ros__parameters:
    base_frame: ""              # 空 = {robot_sn}_base_link
    tcp_frame: ""               # 空 = {robot_sn}_grav_tcp
    observe_pose_xyz: [0.489, -0.252, 0.27]   # 放置点正上方 0.30 m
    observe_pose_rpy: [0.0, 3.1416, 0.1679]
    approach_height: 0.10       # 预抓取点相对抓取点的高度
    grasp_offset: -0.02         # 沿标记法向的补偿；-20 mm 让 TCP 落在立方体中心高度
    place_pose_xyz: [0.48882, -0.25160, -0.03021]   # 示教器读数 + 20 mm（桌面→方块中心）
    place_pose_rpy: [0.0, 3.1416, 0.1679]           # RPY (0, 180, 9.62)°
    place_release_height: 0.05
    grasp_force: 10.0           # N，20 g 物体约 5 倍以上余量
    open_width: 0.07            # m，40 mm 立方体留 30 mm 余量
    close_width: 0.01           # m，空夹时的收拢宽度
    velocity_scaling: 0.10      # 0~1，真机起步值
    acceleration_scaling: 0.10
    detect_retries: 3
    detect_timeout_s: 2.0
    action_timeout_s: 60.0

grasp_perception:
  ros__parameters:
    tag_family: "25h9"          # 对应 cv2.aruco.DICT_APRILTAG_25H9
    tag_size: 0.026             # m
    target_ids: []              # 空 = 不过滤
    image_topic: "/camera/camera/color/image_raw"
    depth_topic: "/camera/camera/aligned_depth_to_color/image_raw"
    camera_info_topic: "/camera/camera/color/camera_info"
    max_age_s: 0.2
    min_area_px: 150            # 过小直接丢弃，避免误检
    max_reproj_error_px: 3.0
```

---

## 8. 感知设计要点

### 8.1 标记与检测

- family `25h9`（AprilTag），边长 26 mm，物体六面均有贴标。
- 检测用 `cv2.aruco.ArucoDetector(DICT_APRILTAG_25H9)` + `solvePnP`（IPPE 或 ITERATIVE），
  已知边长与相机内参即可得到 `T_camera_tag`。
- **不引入 `cv_bridge`**：系统 Python 同时存在 apt 的 OpenCV 4.5.4（`cv_bridge` 链接）与 pip 的
  OpenCV 4.13（本节点使用），同进程加载两份 libopencv 有崩溃风险。节点直接订阅
  `sensor_msgs/Image`，用 `numpy.frombuffer(...).reshape(h, w, 3)` 取 BGR 数据，只依赖 numpy + cv2。
- **v1 不使用深度流**：26 mm 标记的深度采样噪声与空洞风险大于收益，位姿完全由单目 PnP 得到；
  深度对齐留到需要"物体高度估计"时再加（届时用于自动设置 `grasp_offset`）。

### 8.2 六面贴标的处理

- 一次帧内可能同时看到多个面（尤其侧面斜视）。策略：按 `area` 降序 + `reprojection_error` 阈值筛选，
  取**正对相机、面积最大**的那一面作为当前帧的观测；不做多面融合（v1 不需要）。
- 顶抓时，被抓面即朝上的那一面；位姿的 z 轴取标记外法向，`grasp_pose` 的姿态构造为
  "TCP z 轴 = -法向"，yaw 取标记自身朝向，另留 `yaw_offset` 参数供微调。
- 若物体顶面朝上但标记贴歪（手工粘贴误差），可在 YAML 里加 `tag_to_object_rpy` 补偿，v1 先不做。

### 8.3 分辨率与距离约束（重要）

26 mm 标签偏小，投影尺寸估算如下（D415 彩色，640×480，f≈460 px）：

| 观察距离 | 标签像素宽度（约） | 每模块像素（7×7 模块） | 可用性 |
| --- | --- | --- | --- |
| 0.30 m | ~40 px | ~5.7 | 好 |
| 0.40 m | ~30 px | ~4.3 | 可接受 |
| 0.50 m | ~24 px | ~3.4 | 边缘，易丢检 |
| 0.60 m | ~20 px | ~2.9 | 不可靠 |

**结论**：观察位到物体的距离应控制在 **0.30–0.40 m**；图像用 **1280×720** 彩色流可把像素数提高约一倍，
作为推荐配置。此表为几何估算，M2 阶段用实测数据校正。

### 8.4 触发时机

眼在手的图像是连续流，但只在观察位静止时才有意义。因此 `Detect` 设计为**按需服务**：
任务节点到达观察位并等待 `settle_time`（默认 0.5 s，抑制运动模糊）后调用一次；
`image_stale=true` 或检测为空时，按 `detect_retries` 重试，仍失败则返回 `DETECT_FAILED`。

---

## 9. 任务状态机

```
IDLE
 └─(goal)─> PREFLIGHT ── 检查：机器人 operational / 夹爪 ready / TF 可达 / 放置点位姿有效
              │ fail → ABORT(PRECHECK_FAILED)
              ▼
           OBSERVE ── MoveIt 规划到观察位，等待 settle
              ▼
           DETECT ── 调 /detect；为空或 stale 则重试
              │ fail → ABORT(DETECT_FAILED)
              ▼
           APPROACH ── 计算 pre-grasp = grasp - n*approach_height
              │          规划到 pre-grasp（OMPL），再笛卡尔直线到 grasp
              │ fail → ABORT(PLAN_FAILED / EXEC_FAILED)
              ▼
           GRASP ── 调 /grasp(force)；读回宽度，判断是否闭合到位
              │ fail → ABORT(GRASP_FAILED)
              ▼
           LIFT ── 笛卡尔直线抬升 place_release_height
              ▼
           PLACE ── 规划到放置点上方，笛卡尔下降到放置位
              ▼
           RELEASE ── /move(open_width)
              ▼
           RETREAT ── 抬升并回观察位（可选）
              ▼
           SUCCEED
```

错误码（`message` 前缀）：`PRECHECK_FAILED` / `DETECT_FAILED` / `PLAN_FAILED` / `EXEC_FAILED` /
`GRASP_FAILED` / `TIMEOUT` / `CANCELED`。

取消：action 被 cancel 时，停止当前 MoveIt 执行，夹爪保持当前状态（不自动松开，安全优先）。

---

## 10. 里程碑与验收

| 阶段 | 内容 | 验收标准 |
| --- | --- | --- |
| **M0** | 建包骨架、消息/服务/action、launch、状态机空跑（fake hardware） | ✅ **已完成**：`use_fake_hardware:=true` 下走完 OBSERVE→…→RETREAT，返回 `success=true` |
| **M1** | 真机运动与夹爪联调（不抓物） | 观察位往返 10 次无报警；夹爪开合正常，`/grasp` 有力反馈；急停响应正常 |
| **M2** | 视觉链路静态验证 | 手拿标记在相机前 0.3/0.4/0.5 m 处，`/detect` 输出 base 系位置与卷尺实测误差 < 5 mm（0.3–0.4 m） |
| **M3** | 真机抓取闭环 | 🟡 首次抓取成功（2026-10-30）：OBSERVE→…→LIFT 全流程跑通并夹起方块；待修问题：下探时夹爪未先张开（已修）、放置点待确认；重复性测试未做 |
| **M4** | 放置 + 收尾 + 演示脚本 | 完整 pick-and-place 10 次 ≥ 7 次；`docs/` 下有一页操作说明 |

时间估算（不含标定）：M0 半天、M1 半天、M2 半天、M3 一天、M4 半天。

---

## 11. 风险与对策

| 风险 | 影响 | 对策 |
| --- | --- | --- |
| 外参不准 | 抓偏，成功率低 | M2 静态验证先卡 5 mm；外参支持单独重载，不用重编译 |
| 26 mm 标记偏小 | 0.5 m 外丢检、位姿抖动 | 观察距离 ≤0.4 m；用 1280×720；`min_area_px` 过滤 |
| 六面贴标导致多面干扰 | 选错面 → 抓取姿态错 | 取面积最大 + 重投影误差最小；可加 `target_ids` 白名单 |
| 顶抓时夹爪与标记干涉 | 夹到贴纸而非物体 | `grasp_offset` 沿法向补偿；M3 实测标定 |
| MoveIt 首次上真机时序 | move_group 未就绪即发 goal | PREFLIGHT 里检查 action server 可用性 + 超时 |
| Grav 力控参数不当 | 压坏物体 / 打滑 | `grasp_force` 起步 20 N，按物体重量调；失败码区分"未夹到"与"夹到但滑" |
| 相机线缆干涉 | 腕部运动受限/扯断 | 观察位与抓取路径避开极限姿态；M1 时低速验证 |

---

## 12. 待确认 / 待提供

| # | 内容 | 现状 |
| --- | --- | --- |
| 1 | 手眼外参 `T_flange_camera`（xyz + rpy） | 由 `flexiv_handeye_calib` 标定，见 `docs/handeye_calibration.md` |
| 2 | 放置点坐标（base 系 xyz + rpy） | ✅ 已给：示教器读数 z=-0.05021 为桌面高度，`place_pose` 存方块中心 z=-0.03021；rpy (0, 180°, 9.62°) |
| 3 | 物体尺寸与重量 | ✅ 已给：40×40×40 mm 立方体，20 g |
| 4 | 标记 ID 范围 | 待给（可为空 = 不过滤） |
| 5 | 观察位姿 | 暂用派生值（放置点正上方 0.30 m）；M3 若视野不够再示教覆盖 |
| 6 | 标记 family 确认 | ✅ 已确认：AprilTag 25h9（`cv2.aruco.DICT_APRILTAG_25H9`） |
| 7 | 物体形状 | ✅ 已确认：40 mm 立方体，六面贴标 |
| 8 | 放置点的 z 语义 | ✅ 已确认：示教读数是空爪指尖触桌面（= 桌面 -0.05021），`place_pose` 存方块中心 -0.03021 |
| 9 | `grav_tcp` 与示教器/RDK TCP 是否重合 | ✅ 已确认重合，无需补偿；`grasp_offset` 与 `place_pose` 的几何关系成立 |

---

## 13. 本轮明确不做

- 视觉伺服 / 连续位姿跟踪
- 多物体识别与分拣、抓取顺序规划
- 手眼标定工具（**已补做**：`src/flexiv_handeye_calib` + `docs/handeye_calibration.md`）
- 抓取力/位姿自适应学习
- 复杂避障场景建模（碰撞用 MoveIt 默认规划 + 简单桌面场景）
- 与下游 VLA 策略的集成（action 接口预留，后续接）

---

## 14. 当前实现进度

| 项 | 状态 |
| --- | --- |
| 包 `src/flexiv_grasp_demo` | 已建，可编译 |
| `PickPlace.action` / `Detect.srv` / `GraspTarget.msg` | 已实现 |
| C++ 任务节点（状态机 + MoveIt + 夹爪 action client） | 已实现，fake 下全流程通过 |
| Python 感知节点（AprilTag 25h9 + PnP + TF） | 已实现，无相机时返回 `image_stale=true` |
| 假夹爪服务（fake hardware 用） | 已实现 |
| 冒烟测试 `test/smoke_fake_hardware.sh` | 已通过（退出码 0） |
| M1 真机空载联调 | ✅ 已完成（2026-10-30） |
| M3 真机抓取闭环 | 🟡 首次抓取成功；待重复性验证 |

M0 实测记录：OBSERVE→DETECT→APPROACH→GRASP→LIFT→PLACE→RELEASE→RETREAT 全部完成，
`success=true`，`measured_width=0.022 m`（假物体宽 0.02 m），单次耗时约 60 s（假硬件，速度 10%）。

## 15. 交付物

1. `src/flexiv_grasp_demo/`：源码、接口定义、launch、YAML、README。
2. 本文档（持续更新至 v1.0）。
3. 一页操作手册：如何起各节点、如何发 goal、如何看日志与错误码。
