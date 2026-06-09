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
ros2 launch fqplanner_nav_bridge mujoco_navigation.launch.py backend_url:=http://127.0.0.1:5001
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
