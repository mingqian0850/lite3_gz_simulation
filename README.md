# Lite3 Gazebo Simulation

ROS 2 Humble + Gazebo Classic 11 simulation for the Lite3 omnidirectional base equipped with a Livox Mid-360 LiDAR. All files below already live inside `src/lite3_gz_simulation`.

## Repository Layout

```
lite3_sim_description/   # URDF/Xacro assets
lite3_sim_bringup/       # Launch + RViz bringup
modules/
  livox_laser_simulation_ROS2/   # Livox official plugin (git submodule)
```

## 1. Clone & Submodule

```bash
git clone https://github.com/<your-org>/lite3_gz_simulation.git
cd lite3_gz_simulation
git submodule add https://github.com/Livox-SDK/livox_laser_simulation_ROS2.git src/lite3_gz_simulation/modules/livox_laser_simulation_ROS2
git submodule update --init --recursive
```

> Note: The Livox repository builds the custom Gazebo plugin that publishes real Mid-360 scan patterns (`sensor_msgs/PointCloud2`). Keep it inside the `modules/` folder so colcon treats it as part of the workspace.

## 2. Build

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Ensure `livox_laser_simulation_ROS2` compiles first time so that the Livox Gazebo plugin library lands on your `GAZEBO_PLUGIN_PATH`. When using separate terminals, always `source install/setup.bash`.

## 3. Launch Gazebo + RViz

```bash
ros2 launch lite3_sim_bringup lite3_gazebo.launch.py
```

What happens:

- `gazebo_ros` starts an empty world and spawns the `lite3` Xacro model.
- `libgazebo_ros_planar_move.so` consumes `/cmd_vel` and publishes `/odom` + TF `odom -> base_link`, enabling omnidirectional motion.
- `livox_lidar_plugin_ros2` streams `/livox/lidar` (`sensor_msgs/PointCloud2`) using the Mid-360 CSV scan profile.
- `robot_state_publisher` runs in namespace `/lite3` with `use_sim_time=true`.
- RViz2 auto-loads `rviz/lite3_sim.rviz`, already configured for TF, robot model, and Livox point clouds.

## 4. Usage Notes

- Send velocity commands on `/cmd_vel` to drive the base (holonomic planar motion).
- Consume odometry from `/odom` and TF tree `odom -> base_link -> livox_link`.
- Point cloud frame is `livox_link`; adjust SLAM / mapping stacks accordingly.
- The Xacro references `config/Mid360.csv` from the Livox repo. Update the path if the upstream file name changes.
- If RViz is not desired, pass `start_rviz:=false` to the launch file.
- To tweak Livox behavior (publish rate, CSV, etc.), edit `lite3_sim_description/urdf/lite3.xacro`. The plugin accepts CSVs shipped in the Livox submodule under `config/`.

## 5. Extending

- Drop additional sensors into `lite3_sim_description/urdf`.
- Mount navigation stacks by reusing `/lite3` namespace and `/odom` TF.
- Swap worlds by passing standard arguments to `gazebo.launch.py` (e.g. `world:=<path>` via `extra_gazebo_args`).

Enjoy simulating Lite3!
