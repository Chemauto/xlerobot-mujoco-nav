# fqplanner_nav_bridge

`fqplanner_nav_bridge` 用于把 FQPlanner 的 MuJoCo HTTP 后端接入 ROS2 的 `slam_toolbox` 和 Nav2。

## 功能

`mujoco_bridge` 将 MuJoCo 后端接口转换成 ROS2 导航话题：

```text
GET  /base_status  ->  /odom
GET  /scan         ->  /scan
TF                 ->  odom -> base_link -> laser
/cmd_vel           ->  POST /cmd_vel
```

`nav2_goal_bridge` 提供一个兼容 FQPlanner 的 HTTP 入口：

```text
POST /nav          ->  Nav2 NavigateToPose action
其他 HTTP 请求     ->  代理回 MuJoCo 后端
```

也就是说，上层仍然可以通过 `robot_api.client.navigate_to()` 发导航指令，只是底层导航可以切换到 Nav2。

## 构建

```bash
cd /home/fangqi/WorkXCJ/ros2_ws
colcon build --packages-select fqplanner_nav_bridge
source install/setup.bash
```

## 启动 MuJoCo 后端

先启动 FQPlanner MuJoCo 后端：

```bash
cd /home/fangqi/WorkXCJ/FQPlanner_Mujoco/serve
python main.py
```

## SLAM 建图

启动后会同时打开 RViz，可视化 `/scan`、`/odom`、TF 和 `/map`：

```bash
cd /home/fangqi/WorkXCJ/ros2_ws
source install/setup.bash
ros2 launch fqplanner_nav_bridge mujoco_slam.launch.py backend_url:=http://127.0.0.1:5001
```

常用检查：

```bash
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /map --once
```

建图时需要让机器人移动。另开一个终端发布 `/cmd_vel`，bridge 会把它转发到 MuJoCo 后端 `/cmd_vel`。

方式一：键盘控制，推荐：

```bash
cd /home/fangqi/WorkXCJ/ros2_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

如果没有安装：

```bash
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
```

方式二：直接发速度指令：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
-r 5
```

原地转向：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" \
-r 5
```

停止：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
--once
```

保存 SLAM 地图：

```bash
mkdir -p /home/fangqi/WorkXCJ/FQPlanner_Mujoco/nav2/maps/slam
ros2 run nav2_map_server map_saver_cli \
  -f /home/fangqi/WorkXCJ/FQPlanner_Mujoco/nav2/maps/slam/mujoco_slam
```

保存后会得到：

```text
nav2/maps/slam/mujoco_slam.yaml
nav2/maps/slam/mujoco_slam.pgm
```

如果看到类似下面的 warning：

```text
minimum laser range setting ... exceeds the capabilities of the used Lidar
maximum laser range setting ... exceeds the capabilities of the used Lidar
```

这通常不是错误。含义是 `slam_toolbox` 默认激光范围和 bridge 发布的 `/scan` 范围不完全一致。当前 bridge 的最大激光距离是 `5.0 m`，`slam_toolbox` 会自动裁剪。

## Nav2 导航

先确认静态地图存在，必要时重新生成：

```bash
cd /home/fangqi/WorkXCJ/FQPlanner_Mujoco
python nav2/map_generator.py --from-sim
```

启动 Nav2：

```bash
cd /home/fangqi/WorkXCJ/ros2_ws
source install/setup.bash
FQPLANNER_ROOT=/home/fangqi/WorkXCJ/FQPlanner_Mujoco \
ros2 launch fqplanner_nav_bridge mujoco_navigation.launch.py backend_url:=http://127.0.0.1:5001 \
map:=/home/fangqi/WorkXCJ/FQPlanner_Mujoco/nav2/maps/slam/mujoco_slam.yaml
```

启动后会打开 RViz，并且 `mujoco_bridge` 会自动把 MuJoCo 当前底盘位姿发布到 `/initialpose`，用于让 AMCL 建立 `map -> odom`。

如果看到：

```text
Timed out waiting for transform from base_link to map
Invalid frame ID "map" ... frame does not exist
```

说明 Nav2 还没有拿到 `map -> odom -> base_link` 这条 TF。常见原因：

- 没有重新 `colcon build`，新版本 bridge 没有发布 `/initialpose`。
- 静态地图没有加载成功，检查 `/map`。
- AMCL 还没有收到初始位姿，可以在 RViz 里用 `2D Pose Estimate` 手动点一下机器人当前位置。

检查命令：

```bash
ros2 topic echo /map --once
ros2 topic echo /initialpose --once
ros2 run tf2_ros tf2_echo map base_link
```

`nav2_goal_bridge` 默认监听：

```text
http://127.0.0.1:5102
```

如果希望 FQPlanner 的 `navigate_to()` 走 Nav2，而不是直接走 MuJoCo `/nav`，设置：

```bash
export ROBOT_API_URL=http://127.0.0.1:5102
```

也可以修改 `robot_api/config.yaml`：

```yaml
backends:
  mujoco:
    url: "http://127.0.0.1:5102"
```

## 目录

```text
fqplanner_nav_bridge/
├── launch/
│   ├── mujoco_slam.launch.py
│   └── mujoco_navigation.launch.py
├── config/
│   └── nav2_params.yaml
└── fqplanner_nav_bridge/
    ├── mujoco_bridge.py
    └── nav2_goal_bridge.py
```

## 注意

- 这个包不会启动 MuJoCo，需要先启动 `FQPlanner_Mujoco/serve/main.py`。
- `mujoco_bridge` 依赖 MuJoCo 后端提供 `/base_status`、`/scan` 和 `/cmd_vel`。
- `nav2_goal_bridge` 只拦截 `/nav`，其他 HTTP 请求会代理回 MuJoCo 后端。
- 当前 `/scan` 是基于 MuJoCo 占据栅格模拟出来的二维激光，不是真实物理雷达模型。
# xlerobot-mujoco-nav
