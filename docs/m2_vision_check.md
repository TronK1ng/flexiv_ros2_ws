# M2 视觉静态验证

目标：确认「相机 → 手眼外参 → base 系」这条链路给出的位置是准的，**在抓取之前**。

## 为什么不用卷尺

用卷尺量需要知道 base_link 在桌上的投影点，那个点通常在机器人底座内部，量不准。

更好的判据是利用一个物理事实：**方块放在桌上不动，它在 base 系里的坐标就不该变。**
机械臂换姿态、相机跟着动，但只要外参正确，报告出来的方块位置必须保持一致。

所以判定标准变成：

> 在 3 个以上明显不同的机械臂姿态下，同一个静止方块被检测到的 base 系位置，彼此差异 < 5 mm。

这个量直接包含了手眼外参误差 + 标记检测误差，正是我们关心的东西。

## 准备

1. 把 40 mm 立方体放在桌面上，**顶面标记朝上**，位置随便，但要落在机械臂能看清楚的区域
2. 起相机：

   ```bash
   ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
   ```

3. 起机器人栈 + 手眼 TF + **感知节点**（不起任务节点）：

   ```bash
   cd ~/tianJiZhiYuan/flexiv_ros2_ws
   source /opt/ros/humble/setup.bash && source install/setup.bash

   ros2 launch flexiv_grasp_demo grasp_demo.launch.py \
     robot_sn:=Rizon4s-064230 robot_type:=Rizon4s \
     start_task:=false detect_mode:=vision
   ```

4. 确认相机话题对得上：

   ```bash
   ros2 topic hz /camera/camera/color/image_raw
   ```

   没有数据就说明 realsense 的命名空间不是 `/camera/camera`，改 `config/grasp_demo.yaml` 里的 `image_topic`。

## 第一次检测

把机械臂 jog 到一个能看见方块的位置（工具大致朝下，离方块 0.3–0.4 m），然后：

```bash
ros2 run flexiv_grasp_demo detect_probe.py --ros-args -p samples:=5
```

正常输出：

```
sample 1: [+0.4123, -0.0712, -0.0521] m
...
id=7 frame=Rizon4s-064230_base_link area=1489px reproj=0.18px

mean: [+0.4123, -0.0710, -0.0519] m
std : [0.42, 0.31, 0.55] mm  over 5 samples
```

先看三件事：

- `frame` 必须是 `<robot_sn>_base_link`，不是 `camera_color_optical_frame`——如果是后者，说明 TF 没查到，检查 `handeye_tf_publisher` 在不在
- `area` 建议 > 800 px（26 mm 标记在 0.35 m 处大约 1500 px）
- `reproj` 应 < 1 px
- `std` 应该在 1 mm 量级（静止状态下纯检测噪声）

### 绝对高度可以直接核对

桌面高度是已知的（示教器读数 `z = -0.05021`），方块高 40 mm，所以：

| 被检测到的面 | 期望 z |
| --- | --- |
| 顶面（正确） | 约 **-0.010** |
| 侧面（错误） | 约 **-0.030** |

如果测出来在 -0.030 附近，说明拿到的是侧面，不是顶面。感知节点默认只接受法向朝上的标记
（`selection: up_facing`, `min_normal_z: 0.7`），所以正常情况下不应该出现；如果出现了，
检查 `message` 里的 `normal_z`。

这一条是这个阶段最有价值的判据：不需要任何外部基准，只看 z 值就能判断手眼链路有没有系统性偏差。

如果返回 `image_stale=True` 说明帧太旧或相机没起；返回 `no marker visible` 说明视野里没有标记或标记太小。

## 换姿态重复

**移动机械臂到另一个明显不同的姿态**（平移 15 cm 以上，或改变俯仰角），再跑一次同样的命令，
记下新的 `mean`。

至少做 3 个姿态。建议覆盖：

| 姿态 | 相机到方块距离 | 说明 |
| --- | --- | --- |
| A | ~0.30 m | 近距离 |
| B | ~0.40 m | 中距离 |
| C | ~0.35 m，从侧面斜看 | 改变观察角 |

## 判定

把 3 个 `mean` 放在一起比较：

- **各轴差异 < 5 mm** → 通过，可以进 M3
- **5–10 mm** → 可用但偏大，先检查 `reproj` 和 `area` 是否正常，必要时重新标定
- **\> 10 mm**，或者差异随机械臂姿态呈现系统性规律 → 手眼外参有问题，回 `.cache/handeye/` 重新标定

## 可视化辅助

感知节点会发布 `ros2 run image_view` 能看的调试图：

```bash
ros2 run rqt_image_view rqt_image_view /grasp_demo/debug_image
```

画面上标记应该被方框框住、四角贴合。如果框歪了或闪烁，检查标记是否平整、光照是否过曝。

## 通过后

把三个姿态的 `mean` 和 `std` 记到 `docs/grasp_demo_requirements.md` 的 M2 行，然后进 M3：
把方块放好，发一个真实的 goal，看它能不能抓起来。
