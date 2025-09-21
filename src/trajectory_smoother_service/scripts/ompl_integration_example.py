#!/usr/bin/env python3
# 기존 OMPL 노드에 추가할 코드 예시

import rclpy
from rclpy.node import Node
from trajectory_smoother_service.srv import SmoothTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np

class OMPLPlannerWithSmoother(Node):
    def __init__(self):
        super().__init__('ompl_planner_with_smoother')
        
        # 기존 OMPL 관련 초기화 코드...
        
        # Smoother 서비스 클라이언트 추가
        self.smoother_client = self.create_client(SmoothTrajectory, 'smooth_trajectory')
        
        # 서비스 대기
        while not self.smoother_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for trajectory smoother service...')
        
        self.get_logger().info('OMPL Planner with Smoother Ready!')
    
    def plan_path(self, start_joints, goal_joints):
        """
        기존 OMPL planning 함수 수정 버전
        """
        # 1. 기존 OMPL planning 수행
        ompl_waypoints = self.solve_ompl(start_joints, goal_joints)
        self.get_logger().info(f'OMPL found path with {len(ompl_waypoints)} waypoints')
        
        # 2. *** 여기가 새로 추가된 한 줄! ***
        smooth_trajectory = self.smooth_trajectory(ompl_waypoints)
        
        if smooth_trajectory is not None:
            self.get_logger().info(f'Trajectory smoothed to {len(smooth_trajectory.points)} points')
            return smooth_trajectory
        else:
            # Fallback: 원본 사용
            self.get_logger().warn('Using original OMPL waypoints')
            return self.create_simple_trajectory(ompl_waypoints)
    
    def smooth_trajectory(self, waypoints_array):
        """
        Smoother 서비스를 호출하는 함수
        """
        request = SmoothTrajectory.Request()
        
        # waypoints를 서비스 요청 형식으로 변환
        for waypoint in waypoints_array:
            point = JointTrajectoryPoint()
            point.positions = waypoint.tolist()
            request.waypoints.append(point)
        
        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1
        
        # 서비스 호출
        future = self.smoother_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                return response.trajectory
            else:
                self.get_logger().error(f"Smoothing failed: {response.message}")
                return None
        else:
            self.get_logger().error("Smoother service call failed")
            return None
    
    def solve_ompl(self, start, goal):
        """
        기존 OMPL solver (수정 없음)
        """
        # 기존 OMPL planning 코드...
        # 여기서는 더미 데이터 반환
        return np.array([
            start,
            [(start[i] + goal[i]) / 2 for i in range(len(start))],  # 중간점
            goal
        ])
    
    def create_simple_trajectory(self, waypoints):
        """
        Fallback용 간단한 trajectory 생성
        """
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

def main():
    rclpy.init()
    
    planner = OMPLPlannerWithSmoother()
    
    # 테스트
    start_joints = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    goal_joints = [1.0, -0.8, 0.8, -0.8, -0.8, 1.0]
    
    trajectory = planner.plan_path(start_joints, goal_joints)
    
    if trajectory:
        planner.get_logger().info('Planning completed successfully!')
    
    rclpy.spin(planner)
    rclpy.shutdown()

if __name__ == '__main__':
    main()