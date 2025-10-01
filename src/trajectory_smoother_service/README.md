# Trajectory Smoother Service

ROS 2 package that wraps MoveIt's CHOMP optimiser and time-parameterisation
pipeline behind a simple service interface.  Feed a list of joint-space
waypoints, get back a smooth and time-stamped `JointTrajectory`, and (optionally)
publish it to `/smooth_trajectory` for downstream controllers.

## Features

- Uses MoveIt's CHOMP (`chomp_motion_planner`) to optimise coarse waypoint sets.
- Applies Time-Optimal Trajectory Generation (TOTG) with Parabolic fallback to enforce velocity/acceleration limits.
- Publishes the smoothed trajectory when requested, enabling quick visualisation in RViz or logging tools.
- Includes test scripts and launch files for integration with the UR Gazebo simulation.
- Respects joint velocity/acceleration limits from the robot model, with scaling knobs for additional safety margins.

## Dependencies

Tested on **ROS 2 Humble** with MoveIt.  Install the required MoveIt packages
alongside CHOMP support:

```bash
sudo apt update
sudo apt install \
  ros-humble-moveit \
  ros-humble-chomp-motion-planner \
  ros-humble-moveit-planners-chomp
```

If you want to run the plotting helper in
`scripts/test_smoother_client.py`, install the optional Python tools:

```bash
sudo apt install python3-matplotlib python3-numpy
```

## Building

From the root of your workspace (e.g. `~/workspaces/ros-ur-gz-ruis`):

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select trajectory_smoother_service
source install/setup.bash
```

## Launching with UR Gazebo + MoveIt

1. Start the UR Gazebo simulation:
   ```bash
   ros2 launch ur_simulation_gz ur_sim_control.launch.py
   ```

2. In another terminal, bring up MoveIt and the smoother service:
   ```bash
   source install/setup.bash
   ros2 launch trajectory_smoother_service smoother_with_moveit.launch.py \
       launch_rviz:=true launch_servo:=false
   ```

   This launch file includes `ur_moveit.launch.py` from `ur_moveit_config`, so
   the base simulation only needs to be started once.

## Testing the Service

Run the bundled client after sourcing your workspace:

```bash
python3 src/trajectory_smoother_service/scripts/test_smoother_client.py
```

The script waits for `/smooth_trajectory`, calls the service with a small set of
waypoints, and (if Matplotlib is available) plots the raw vs. smoothed
positions, velocities, and accelerations.

## Service API

```
service /smooth_trajectory : trajectory_smoother_service/srv/SmoothTrajectory

Request:
  trajectory_msgs/JointTrajectoryPoint[] waypoints
  float64 max_velocity_scaling_factor
  float64 max_acceleration_scaling_factor
  float64 totg_resample_dt
  float64 hermite_resample_dt

Response:
  trajectory_msgs/JointTrajectory trajectory
  bool success
string message
```

Provide joint positions (velocities/accelerations optional) for each waypoint.
Set the scaling factors (0 < value ≤ 1) to derate the robot limits used during
timing.  Set `totg_resample_dt` to a positive value if you want TOTG to resample
at that interval, and `hermite_resample_dt` to control the output spacing; leave
either ≤ 0 to use the defaults from `smoother_parameters.yaml`.

## Joint Limits & Kinematics

- MoveIt reads joint limits from the URDF and any `joint_limits.yaml`. For UR robots the
  default file resides in `/opt/ros/humble/share/ur_moveit_config/config/joint_limits.yaml`.
- To version-control custom limits, copy that file under `trajectory_smoother_service/config/`
  and point the launch argument `moveit_joint_limits_file` to your copy.
- The smoother multiplies those limits by `max_velocity_scaling_factor` and
  `max_acceleration_scaling_factor` (defaults in `smoother_parameters.yaml`). Lower them for
  gentler execution.
- If no kinematics parameters are supplied, MoveIt falls back to the KDL solver and logs a
  warning. To use the UR-specific IK plugin, include `config/kinematics.yaml` in the smoother’s
  parameters (copy from `ur_moveit_config` or reference the installed file).

## Post-processing Tips

- CHOMP may densify the waypoint list internally (the response often contains many more points
  than provided). If your controller requires samples at a fixed interval, resample the returned
  trajectory to your servo period while keeping positions/velocities/accelerations consistent.
- To stretch the total duration beyond the limits, either reduce the scaling factors or
  post-multiply every `time_from_start` by a constant > 1 after TOTG runs.

## Integrating with an OMPL Planner

Most OMPL planners return a `robot_trajectory::RobotTrajectory` that can be
converted directly into the request.  Use the raw OMPL states (start, user
waypoints, and any extra states inserted to avoid obstacles) without additional
interpolation—the smoother will refine and densify the path for you.

### Minimal Python Client Example

```python
import rclpy
from rclpy.node import Node
from trajectory_smoother_service.srv import SmoothTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_smoother_bridge')
        self.cli = self.create_client(SmoothTrajectory, 'smooth_trajectory')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for smoother...')

    def smooth_path(self, waypoint_matrix, v_scale=0.2, a_scale=0.2):
        req = SmoothTrajectory.Request()
        for joint_positions in waypoint_matrix:
            point = JointTrajectoryPoint()
            point.positions = list(joint_positions)
            req.waypoints.append(point)
        req.max_velocity_scaling_factor = v_scale
        req.max_acceleration_scaling_factor = a_scale

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    node = PlannerNode()

    # Example: start + user waypoints + goal (no interpolation)
    waypoints = [
        [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
        [0.5, -1.2, 1.2, -1.2, -1.2, 0.5],
        [1.0, -0.8, 0.8, -0.8, -0.8, 1.0],
    ]
    resp = node.smooth_path(waypoints)
    if resp and resp.success:
        node.get_logger().info('Smoothed trajectory ready!')
        # Publish to your controller, or relay to the user project here
    else:
        node.get_logger().error(f'Smoothing failed: {resp.message}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Adapt `waypoints` to the path your OMPL planner produced—typically the start
state, mandatory via points, and goal state (possibly with additional states the
planner added around obstacles).  Avoid pre-interpolating the segments; let the
smoother handle path refinement and timing.

---

For more examples, see the scripts in `scripts/` within this package.
