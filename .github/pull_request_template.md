# Summary
- Add FT sensor (`tcp_ft_sensor`) on `wrist_3_joint`
- Expose sensor state interfaces in `ros2_control`
- Configure `force_torque_sensor_broadcaster`
- Add Gazebo world plugins (sensors, forcetorque, ros2_control)
- Launch file spawner + ROS–GZ bridge for Wrench topic

# Changes
- `src/ur_description/urdf/ur_macro.xacro`  
  - Added `<gazebo><sensor type="force_torque">` under `wrist_3_joint`
- `src/ur_description/urdf/ur.ros2_control.xacro`  
  - Added `tcp_ft_sensor` state interfaces (force/torque)
- `src/ur_simulation_gz/config/ur_controllers.yaml`  
  - Configured `force_torque_sensor_broadcaster`
- `src/ur_simulation_gz/worlds/empty_with_ft.sdf`  
  - Added Gazebo plugins (`sensors`, `forcetorque`, `ign_ros2_control`)
- `src/ur_simulation_gz/launch/ur_sim_control.launch.py`  
  - Spawner for FT broadcaster  
  - Parameter bridge for `/tcp_ft_sensor/forcetorque`

# How to test
\`\`\`bash
rm -rf build install log
colcon build
source install/setup.bash

ros2 launch ur_simulation_gz ur_sim_control.launch.py gazebo_gui:=true

ros2 topic list | grep wrench
ros2 topic echo /tcp_ft_sensor/forcetorque
\`\`\`

# Notes
- `frame_id`: `ft_frame`
- Topic name may be remapped to `/ur5e/ft_wrench` if desired
- Requires `ros-humble-ros-gz-bridge`, `ros-humble-force-torque-sensor-broadcaster`

# Checklist
- [ ] Builds locally (`colcon build`)
- [ ] Simulation runs without errors
- [ ] Wrench data published on expected topic
- [ ] README updated with FT sensor setup
