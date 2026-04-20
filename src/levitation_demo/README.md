
# levitation_demo

A ROS 2 Foxy + Gazebo Classic demo package for a hanging mobile module under a fixed plate.

## What this package does

- Spawns your uploaded `levitation_robot.urdf`
- Spawns a fixed top plate named `copper_plate`
- Uses a custom Gazebo Classic model plugin to:
  - hold the robot under the plate with a simple vertical controller
  - lock roll / pitch / yaw with restoring torques
  - move the robot in X and Y with force commands from ROS 2
- Provides a simple keyboard teleop node

## Assumptions used

- Plate pose: `(-0.510715, 0.350583, 2.0)`
- Plate size: `4.0 x 6.0 x 0.005 m`
- Controlled body link: `base_link`
- Target hanging gap: `0.02 m`
- Top surface of `base_link` is `0.0025 m` above the link origin

## Build

```bash
cd ~/your_ws/src
cp -r /path/to/levitation_demo .
cd ..
source /opt/ros/foxy/setup.bash
colcon build --packages-select levitation_demo
source install/setup.bash
```

## Run

```bash
ros2 launch levitation_demo levitation_demo.launch.py
```

In another terminal:

```bash
source /opt/ros/foxy/setup.bash
source ~/your_ws/install/setup.bash
ros2 run levitation_demo teleop_force_keyboard.py
```

## Keyboard teleop

- `w`: +X
- `s`: -X
- `a`: +Y
- `d`: -Y
- `x`: stop
- `q`: quit

## Most likely tuning points

Inside `urdf/levitation_robot.urdf`, in the levitation plugin block:

- `target_gap`
- `z_kp`
- `z_kd`
- `z_force_max`
- `planar_force_gain`
- `planar_damping`
- `planar_force_max`
- `rot_kp`
- `rot_kd`

## Notes

- This is the **A-model** version: one virtual holding force acting on one rigid body.
- Your later **B-model** can be implemented by splitting the vertical force into four virtual magnetic pads and adding differential force / torque terms.
- If the robot starts too far from the plate, increase `spawn_z` or raise `z_force_max`.
