# UR5e FT Sensor in Ignition Gazebo + ROS2 Humble

## Modified files:
1. ur_macro.xacro
   - Added <gazebo> <sensor type="force_torque"> on wrist_3_joint

2. ur.ros2_control.xacro
   - Added same <gazebo> sensor definition for ros2_control

3. ur_controllers.yaml
   - Added force_torque_sensor_broadcaster parameters

4. ur_sim_control.launch.py
   - Added spawner for force_torque_sensor_broadcaster (delayed after joint_state_broadcaster)

5. empty.sdf
   - Added plugin:
     - `ignition-gazebo-sensors-system`
     - `ignition-gazebo-forcetorque-system`
     - `ign_ros2_control-system` (class: ign_ros2_control::IgnitionROS2ControlPlugin)
