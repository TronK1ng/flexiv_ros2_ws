# M1 真机联调步骤

目标：在**不接触物体、不靠近桌面**的前提下，确认「驱动 + MoveIt + 夹爪」这条链在真机上不出事。
视觉不参与，相机可以先不开。

## 0. 上机前

- [ ] 急停拍在右手边，人站在能同时看到机器人和急停的位置
- [ ] 工作台清空，机器人运动范围内没有杂物
- [ ] 机器人已 Enable、切到 Remote 模式、无 fault
- [ ] 速度按 5% 起步（下面命令里显式传 `velocity_scaling`）

## 1. 起栈（不启动任务节点）

```bash
cd ~/tianJiZhiYuan/flexiv_ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch flexiv_grasp_demo grasp_demo.launch.py \
  robot_sn:=Rizon4s-064230 robot_type:=Rizon4s \
  start_task:=false detect_mode:=mock start_camera:=false
```

`start_task:=false` 只起驱动、MoveIt、手眼 TF、RViz，不起任务节点——任务节点等一下单独起，
方便用命令行覆盖参数做空载测试。`detect_mode:=mock` 让这个 launch 不起感知节点，`start_camera:=false` 不起相机（M1 两者都用不到）。

确认 RViz 里机械臂模型姿态和真机一致（不一致先别动，查 URDF/SN）。

## 2. 预检（只读，不发任何运动指令）

```bash
bash src/flexiv_grasp_demo/test/m1_preflight.sh Rizon4s-064230
```

它会检查：核心节点、控制器是否 active、action server、手眼 TF 节点，并打印
`/tcp_pose`（RDK，等于示教器读数）和 TF 的 `grav_tcp`（MoveIt 视角）两组位姿。

> 必须在**和起栈终端相同的 ROS 环境**里跑：同一个 `RMW_IMPLEMENTATION`、同一个
> `ROS_DOMAIN_ID`。脚本不替你做选择，头部会打印它当前看到的 RMW 和 domain。
>
> 此时任务节点还没起，所以 `/grasp_task` 和 `/grasp_demo/pick_place` 会是 `[warn]` 而不是
> `[FAIL]`——这是预期的。

**输出 `PRE-FLIGHT PASSED` 才能继续。** 两组位姿的平移应该只差几毫米；如果差得多，
先停下来查工具定义，别往下走。

## 3. 空载动作测试

用 mock 检测，但把抓取点和放置点都**抬高到桌面上方 0.2 m 以上**，让机械臂只在空中走完整流程：

```bash
ros2 run flexiv_grasp_demo grasp_task_node --ros-args \
  -p robot_sn:=Rizon4s-064230 \
  -p detect_mode:=mock \
  -p "mock_tag_xyz:=[0.45, 0.0, 0.30]" \
  -p "place_pose_xyz:=[0.40, 0.15, 0.35]" \
  -p "observe_pose_xyz:=[0.45, 0.0, 0.45]" \
  -p grasp_force:=5.0 \
  -p velocity_scaling:=0.05 \
  -p acceleration_scaling:=0.05
```

另开终端发 goal：

```bash
ros2 action send_goal /grasp_demo/pick_place flexiv_grasp_demo/action/PickPlace "{}" --feedback
```

预期：`OBSERVE → DETECT → APPROACH → GRASP → LIFT → PLACE → RELEASE → RETREAT`，
约 60–90 秒，动作平滑、无报警、无碰撞检测触发。

### 中止条件

出现下列任一情况**立刻拍急停**，不要等它跑完：

- 机械臂抖动、异响、或轨迹明显不是直线
- 控制器报错 / 机器人进入 fault
- 夹爪或线缆要碰到东西
- 任何你拿不准的动作

## 4. 夹爪单独验证

```bash
ros2 action send_goal /flexiv_gripper_node/move flexiv_msgs/action/Move \
  "{width: 0.07, velocity: 0.05, max_force: 10.0}"
ros2 action send_goal /flexiv_gripper_node/move flexiv_msgs/action/Move \
  "{width: 0.02, velocity: 0.05, max_force: 10.0}"
```

确认：开合到位、有实际的宽度反馈、没有卡顿。再试一次力控抓取（空夹）：

```bash
ros2 action send_goal /flexiv_gripper_node/grasp flexiv_msgs/action/Grasp "{force: 5.0}"
```

空夹时应该返回 `success: false`（"closed on nothing" 之类），这是正常的——说明力控在工作。

## 5. 通过标准

- [ ] preflight 全绿，两组位姿平移差 < 5 mm
- [ ] 空载动作完整跑完，`success: true`
- [ ] 全程无 fault、无碰撞检测、无异常声音
- [ ] 夹爪开合与力控都正常

## 6. 记录

把这次的 `velocity_scaling`、实际耗时、是否有异常记到 `docs/grasp_demo_requirements.md`
的 M1 行里，M2/M3 会用到。
