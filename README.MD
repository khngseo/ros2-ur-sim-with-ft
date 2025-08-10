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

# UR5e + Gazebo (Ignition/Fortress) + ROS 2 Humble — Force/Torque Sensor Setup

UR5e 시뮬레이션(`ur_simulation_gz`)에서 **Force/Torque(FT) 센서**를 추가하고 ROS2 토픽으로 값을 읽어내는 방법을 정리했습니다.  
핵심은 **wrist_3 쪽(ft_frame)에 센서 추가**, **Gazebo 플러그인 활성화**, **ROS↔GZ 브릿지 설정**입니다.

---

## 1) 사전 준비 (의존성 설치)

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-gz-ros2-control \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-force-torque-sensor-broadcaster
```

> 이미 설치되어 있는 경우가 많습니다. 없는 것만 설치하면 됩니다.

---

## 2) 워크스페이스 빌드 & 환경 설정

```bash
cd ~/workspaces/ros-ur-gz-ruis
colcon build
source install/setup.bash
```

---

## 3) 변경 사항

아래 변경을 통해 센서가 시뮬레이터에서 계산되고, ROS 토픽으로 브릿지됩니다.

### 3-1) `ur_description/urdf/ur_macro.xacro` (센서 추가)

```xml
  <!-- Ignition Gazebo F/T sensor -->
  <gazebo reference="${tf_prefix}wrist_3_joint">
    <sensor name="tcp_ft_sensor" type="force_torque">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <force_torque>
	<frame>sensor</frame>
        <measure_direction>child_to_parent</measure_direction>
      </force_torque>
    </sensor>                                                                                        
                                                                                                     
```

---

### 3-2) `ur_description/urdf/ur.ros2_control.xacro` (ros2_control 인터페이스 추가)

```xml
<sensors>
  <sensor name="tcp_ft_sensor">
    <state_interface name="force.x"/>
    <state_interface name="force.y"/>
    <state_interface name="force.z"/>
    <state_interface name="torque.x"/>
    <state_interface name="torque.y"/>
    <state_interface name="torque.z"/>
  </sensor>
</sensors>
```

---

### 3-3) `ur_simulation_gz/config/ur_controllers.yaml` (컨트롤러 설정)

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    force_torque_sensor_broadcaster:
      type: force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

force_torque_sensor_broadcaster:
  ros__parameters:
    sensor_name: tcp_ft_sensor
    state_interface_names:
      - tcp_ft_sensor/force.x
      - tcp_ft_sensor/force.y
      - tcp_ft_sensor/force.z
      - tcp_ft_sensor/torque.x
      - tcp_ft_sensor/torque.y
      - tcp_ft_sensor/torque.z
    frame_id: ft_frame
```

---

### 3-4) `ur_simulation_gz/worlds/empty_with_ft.sdf` (World 플러그인 설정)

```xml
<world name="empty">
  <plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="ignition-gazebo-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="ignition-gazebo-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="ignition-gazebo-contact-system" name="gz::sim::systems::Contact"/>
  <plugin filename="ignition-gazebo-forcetorque-system" name="ignition::gazebo::systems::ForceTorque"/>
  <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors"/>
  <plugin filename="ign_ros2_control-system" name="ign_ros2_control::IgnitionROS2ControlPlugin"/>
</world>
```

---

### 3-5) `ur_simulation_gz/launch/ur_sim_control.launch.py` (런치 설정)

```python
# FT 센서 브로드캐스터
ft_sensor_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["force_torque_sensor_broadcaster", "-c", "/controller_manager"],
    output="screen",
)

# ROS <-> GZ 브릿지
gz_sim_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    arguments=[
        "/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock",
        "/world/empty/model/ur/joint/wrist_3_joint/sensor/tcp_ft_sensor/forcetorque"
        "@geometry_msgs/msg/WrenchStamped@ignition.msgs.Wrench",
    ],
    output="screen",
)
```

---

## 4) 실행 방법

```bash
source /opt/ros/humble/setup.bash
source ~/workspaces/ros-ur-gz-ruis/install/setup.bash

ros2 launch ur_simulation_gz ur_sim_control.launch.py gazebo_gui:=true
```

---

## 5) 동작 확인

```bash
ros2 topic list | grep -i forcetorque
ros2 topic echo /world/empty/model/ur/joint/wrist_3_joint/sensor/tcp_ft_sensor/forcetorque
```

(브로드캐스터 사용 시)
```bash
ros2 topic list | grep force_torque_sensor_broadcaster
ros2 topic echo /force_torque_sensor_broadcaster/wrench
```

---

## 6) 수정한 파일 목록

- `ur_description/urdf/ur_macro.xacro`
- `ur_description/urdf/ur.ros2_control.xacro`
- `ur_simulation_gz/config/ur_controllers.yaml`
- `ur_simulation_gz/worlds/empty_with_ft.sdf`
- `ur_simulation_gz/launch/ur_sim_control.launch.py`
