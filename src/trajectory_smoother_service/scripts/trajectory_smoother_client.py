#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_smoother_service.srv import SmoothTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np

class TrajectorySmoothClient:
    def __init__(self, node):
        self.node = node
        self.client = node.create_client(SmoothTrajectory, 'smooth_trajectory')
        
        # 서비스가 준비될 때까지 대기
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for trajectory smoother service...')
    
    def smooth_trajectory(self, waypoints_array, max_vel_scale=0.1, max_acc_scale=0.1):
        """
        waypoints_array: numpy array of shape (n_waypoints, n_joints)
        Returns: trajectory_msgs/JointTrajectory or None if failed
        """
        request = SmoothTrajectory.Request()
        
        # waypoints를 JointTrajectoryPoint로 변환
        for waypoint in waypoints_array:
            point = JointTrajectoryPoint()
            point.positions = waypoint.tolist()
            # velocities와 accelerations는 비워둠 (smoother가 계산)
            request.waypoints.append(point)
        
        request.max_velocity_scaling_factor = max_vel_scale
        request.max_acceleration_scaling_factor = max_acc_scale
        
        # 서비스 호출
        future = self.client.call_async(request)
        
        # 블로킹 방식으로 결과 대기
        rclpy.spin_until_future_complete(self.node, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.node.get_logger().info(f"Trajectory smoothed: {response.message}")
                return response.trajectory
            else:
                self.node.get_logger().error(f"Smoothing failed: {response.message}")
                return None
        else:
            self.node.get_logger().error("Service call failed")
            return None

# 기존 OMPL 노드에 추가할 함수
def smooth_ompl_path(node, waypoints):
    """
    기존 OMPL 노드에서 호출할 함수
    waypoints: numpy array of joint positions
    """
    smoother_client = TrajectorySmoothClient(node)
    smoothed_trajectory = smoother_client.smooth_trajectory(waypoints)
    
    if smoothed_trajectory is not None:
        return smoothed_trajectory
    else:
        # Fallback: 원본 waypoints를 simple trajectory로 변환
        node.get_logger().warn("Using original OMPL path (smoothing failed)")
        return create_simple_trajectory(waypoints)

def create_simple_trajectory(waypoints):
    """Fallback: 간단한 trajectory 생성"""
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    
    trajectory = JointTrajectory()
    trajectory.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", 
                             "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    
    for i, waypoint in enumerate(waypoints):
        point = JointTrajectoryPoint()
        point.positions = waypoint.tolist()
        point.time_from_start = Duration(sec=i, nanosec=0)
        trajectory.points.append(point)
    
    return trajectory

# 테스트용 main
if __name__ == '__main__':
    rclpy.init()
    node = Node('test_smoother_client')
    
    # 테스트 waypoints
    test_waypoints = np.array([
        [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
        [0.5, -1.2, 1.2, -1.2, -1.2, 0.5],
        [1.0, -0.8, 0.8, -0.8, -0.8, 1.0]
    ])
    
    try:
        result = smooth_ompl_path(node, test_waypoints)
        if result:
            node.get_logger().info(f"Success! Trajectory has {len(result.points)} points")
        else:
            node.get_logger().error("Failed to smooth trajectory")
    finally:
        rclpy.shutdown()