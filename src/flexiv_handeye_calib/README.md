# flexiv_handeye_calib

Flexiv Rizon 4s + 腕部相机（eye-in-hand）手眼标定，ROS 2 Humble，无 `cv_bridge` 依赖。

- 采样：从 tf 读 `T_base_flange`，从标靶检测得到 `T_camera_target`
- 标靶：AprilTag/ArUco（`target_type:=apriltag`）或棋盘格（`target_type:=chessboard`，如 DFOPTIX Q12-100-6）
- 求解：`A·X·B = Y`，OpenCV 五种解法 + numpy 非线性精化 + 外点剔除
- 评估：拟合残差、留一交叉验证、多解法一致性、旋转多样性
- 输出：`static_transform_publisher` 可直接使用的 YAML（xyz + rpy + 四元数 + 4×4）
- 验证：独立位姿复现、单帧重投影、tf 链自检（`validate` / `doctor`）

## 快速开始

```bash
# 0) 自检：帧、内参、标靶是否可见
ros2 run flexiv_handeye_calib doctor --ros-args -p robot_sn:=Rizon4s-123456
#    tag 用户：-p target_type:=apriltag -p tag_size:=0.026

# 1) 启动（默认同时拉起 MoveIt 和 RealSense，可按需关闭）
ros2 launch flexiv_handeye_calib calibrate.launch.py robot_sn:=Rizon4s-123456 \
    tag_size:=0.026 auto_capture:=true

# 棋盘格（12x9 方格 -> 8x11 内角点，6 mm）
ros2 launch flexiv_handeye_calib calibrate.launch.py robot_sn:=Rizon4s-123456 \
    target_type:=chessboard chessboard_cols:=8 chessboard_rows:=11 \
    square_size:=0.006 auto_capture:=true

# 2) 手动把机械臂摆 15~20 个不同姿态（RViz 拖动 / 示教 / 手动拖动）
#    auto_capture=true 时停下就自动采样
ros2 service call /handeye_calibrator/status std_srvs/srv/Trigger
ros2 service call /handeye_calibrator/capture std_srvs/srv/Trigger   # 或手动采

# 3) 求解（自动写 YAML 并发布 static tf）
ros2 service call /handeye_calibrator/solve std_srvs/srv/Trigger

# 4) 自动摆位（可选，先试 max_poses:=2）
ros2 run flexiv_handeye_calib auto_move --ros-args -p robot_sn:=Rizon4s-123456 -p max_poses:=2
ros2 service call /auto_move/run std_srvs/srv/Trigger

# 5) 离线复算（可选）
ros2 run flexiv_handeye_calib handeye_solve --samples ~/.ros/flexiv_handeye/samples.json
```

输出默认在 `~/.ros/flexiv_handeye/handeye_calibration.yaml`，可用
`output_yaml:=/path/to/file.yaml` 指定（建议直接写进 `src/flexiv_grasp_demo/config/`）。

## 服务

| 服务 | 作用 |
| --- | --- |
| `/handeye_calibrator/capture` | 采一个样本（tag 不可见/离已有位姿太近会拒绝） |
| `/handeye_calibrator/solve` | 求解 + 打印质量报告 + 写 YAML + 发布 tf |
| `/handeye_calibrator/save` | 重写上一次结果的 YAML |
| `/handeye_calibrator/reset` | 清空样本 |
| `/handeye_calibrator/status` | 当前状态与上次报告 |

## 主要参数

见 `config/handeye.yaml`。最常改的几项：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `target_type` | `chessboard` | `chessboard` 或 `apriltag`（用 25h9 标签时改） |
| `tag_size` | `0.026` | **黑色方块外沿边长（含黑边，不含白边）**，用卡尺量 |
| `chessboard_cols` / `chessboard_rows` | `8` / `11` | **内角点**数，不是方格数 |
| `square_size` | `0.006` | 棋盘格方格边长（m） |
| `chessboard_auto` | `true` | 标靶规格不确定时自动扫描常见规格 |
| `normalize_symmetric_board` | `true` | 自动纠正方板 90° 对称造成的位姿翻转 |
| `tag_family` | `25h9` | 对应 `cv2.aruco.DICT_APRILTAG_25H9` |
| `camera_frame` | `camera_link` | 标定结果的子帧，即 demo 里 static tf 的子帧 |
| `auto_capture` | `false` | 机械臂停稳且移动足够大时自动采样 |
| `solve_method` | `PARK` | `PARK/TSAI/HORAUD/ANDREFF/DANIILIDIS/PARK_LS` |
| `publish_tf` | `true` | demo 里自己发同名 tf 时置 `false` |

## 测试

```bash
colcon test --packages-select flexiv_handeye_calib
# 或
cd src/flexiv_handeye_calib && PYTHONPATH=. python3 -m pytest test -q
```

合成数据测试覆盖：几何往返、五种解法的真值复原、噪声下的精度、留一验证、
外点剔除、退化数据拒绝，以及用渲染图验证 PnP/`tag_size` 约定。

完整流程、精度验证方法与误差预算见 [`docs/handeye_calibration.md`](../../docs/handeye_calibration.md)。
