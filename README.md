# go2_user_tools

ROS 2 tools for GO2 URDF visualization, `/lf/lowstate` joint forwarding, and camera depth point cloud to UniLiDAR ICP extrinsic matching.

Repository/package name:

```text
go2_user_tools
```

## Workspace

Put this package and [`go2_description`](https://github.com/Unitree-Go2-Robot/go2_description) under the same workspace `src/`:

```text
urdf_ws/src/
├── go2_description/
└── go2_user_tools/
```

Build from the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

[`go2_description`](https://github.com/Unitree-Go2-Robot/go2_description) provides the URDF and meshes. `unitree_go/msg/LowState` must also be available in the sourced ROS environment.

## RViz and Joint States

Start GO2 URDF visualization:

```bash
ros2 launch go2_user_tools rviz_view.launch.py
```

This starts `robot_state_publisher`, RViz, and `scripts/lowstate_to_jointstate.py`.

`lowstate_to_jointstate.py` subscribes:

```text
/lf/lowstate
```

and publishes:

```text
/joint_states
```

Joint order:

```text
0 FR_hip    1 FR_thigh    2 FR_calf
3 FL_hip    4 FL_thigh    5 FL_calf
6 RR_hip    7 RR_thigh    8 RR_calf
9 RL_hip   10 RL_thigh   11 RL_calf
```

## Camera TF Rough Value

First publish a rough static TF from the robot base to the physical camera link:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.125604 \
  --y 0.123118 \
  --z 0.278199 \
  --roll -0.013684 \
  --pitch -0.005365 \
  --yaw 0.030555 \
  --frame-id base_link \
  --child-frame-id camera_link
```

Only publish `base_link -> camera_link`. Let the camera driver publish its internal frames:

```text
camera_link -> camera_depth_frame -> camera_depth_optical_frame
```

Do not also publish `base_link -> camera_depth_optical_frame`, or TF will have duplicate parents for the optical frame.

## ICP Calibration

Run:

```bash
ros2 launch go2_user_tools icp_extrinsic.launch.py
```

Inputs:

```text
/camera/camera/depth/color/points
/utlidar/cloud_base
```

Filtered outputs:

```text
/icp/filtered_camera_points
/icp/filtered_lidar_points
```

ICP outputs:

```text
/icp/aligned_camera_points
/icp/extrinsic_transform
```

The ICP result is printed in the terminal:

```text
[ICP RESULT] xyz=(...) rpy=(...) score=... quality=...
Matrix:
  ...
```

## Parameters to Edit

Edit [launch/icp_extrinsic.launch.py](launch/icp_extrinsic.launch.py).

Camera/LiDAR filtering range and downsampling:

```python
'camera_accum_frames': 0,
'camera_voxel_leaf_size': 0.05,
'camera_x_min': -2.0,
'camera_x_max': 2.0,
'camera_y_min': -0.5,
'camera_y_max': 0.8,
'camera_z_min': 0.0,
'camera_z_max': 3.0,

'lidar_accum_frames': 30,
'lidar_voxel_leaf_size': 0.0,
'lidar_x_min': 1.0,
'lidar_x_max': 4.0,
'lidar_y_min': -2.0,
'lidar_y_max': 2.0,
'lidar_z_min': -0.5,
'lidar_z_max': 4.0,
```

ICP initial value:

```python
'target_frame': 'base_link',
'source_frame': 'camera_depth_optical_frame',
'initial_x': 0.182758,
'initial_y': 0.060812,
'initial_z': 0.202804,
'initial_roll': -1.548191,
'initial_pitch': 0.014885,
'initial_yaw': -1.508523,
```

`initial_*` is `base_link -> camera_depth_optical_frame`, not `base_link -> camera_link`. If your rough measurement is `base_link -> camera_link`, convert it through the camera driver's internal TF before filling `initial_*`.

Set voxel size to `0.0` to disable voxel downsampling. `camera_accum_frames: 0` means no accumulation. `lidar_accum_frames: 30` means publish after accumulating 30 LiDAR frames.

## Final Static TF

After ICP, choose the final extrinsic and publish the static TF as:

```text
base_link -> camera_link
```

Example:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.125604 \
  --y 0.123118 \
  --z 0.278199 \
  --roll -0.013684 \
  --pitch -0.005365 \
  --yaw 0.030555 \
  --frame-id base_link \
  --child-frame-id camera_link
```

## Debug

```bash
ros2 topic hz /icp/filtered_camera_points
ros2 topic hz /icp/filtered_lidar_points
ros2 topic echo /icp/extrinsic_transform
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```
