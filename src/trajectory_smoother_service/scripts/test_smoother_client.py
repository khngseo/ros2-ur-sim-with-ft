#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_smoother_service.srv import SmoothTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

try:
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover
    plt = None

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None

def lerp(a, b, t): return [ai*(1-t)+bi*t for ai, bi in zip(a,b)]


def test_smoother():
    rclpy.init()
    node = Node('test_smoother_client')
    
    # 서비스 클라이언트 생성
    client = node.create_client(SmoothTrajectory, 'smooth_trajectory')
    
    # 서비스 대기
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for trajectory smoother service...')
    
    # 테스트 요청 생성
    request = SmoothTrajectory.Request()
    
    # # 테스트 waypoints
    # test_waypoints = [
    #     [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
    #     [0.5, -1.4, -.1, -1.2, -1.2, 0.5],
    #     [1.0, -0.8, 0.8, -0.8, -0.8, 1.0]
    # ]

    # # 의도적으로 베이스 근처를 스치게 하는 짧고 ‘빡빡한’ 구간
    # A = [0.00, -2.00,  2.10, -1.80, -1.57, 0.00]
    # B = [0.35, -1.25,  1.60, -1.50, -1.57, 0.00]

    A = [0.0, -1.57, 1.56, -1.57, -1.57, 0.0]
    B = [0.0, -1.77, 1.76, -1.37, -1.57, 0.0]
    C = [0.0, -1.57, 1.76, -1.77, -1.27, 0.0]
    # 4점(중간 2개) 경로 생성
    P = [A, lerp(A,B,1/3), lerp(A,B,2/3), B, C]

    # 요청에 넣기
    test_waypoints = P #[p for p in P]

    for waypoint in test_waypoints:
        point = JointTrajectoryPoint()
        point.positions = waypoint
        request.waypoints.append(point)
    
    request.max_velocity_scaling_factor = 0.1
    request.max_acceleration_scaling_factor = 0.1
    request.totg_resample_dt = 0.008
    request.hermite_resample_dt = 0.002
    
    # 서비스 호출
    node.get_logger().info('Calling smooth_trajectory service...')
    future = client.call_async(request)
    
    rclpy.spin_until_future_complete(node, future)
    
    if future.result() is None:
        node.get_logger().error('Service call failed!')
        rclpy.shutdown()
        return

    response = future.result()
    if response.success:
        node.get_logger().info(f'SUCCESS: {response.message}')
        node.get_logger().info(f'Received trajectory with {len(response.trajectory.points)} points')

        times = []
        pos_series = [[] for _ in range(len(response.trajectory.joint_names))]
        vel_series = [[] for _ in range(len(response.trajectory.joint_names))]
        acc_series = [[] for _ in range(len(response.trajectory.joint_names))]

        for i, point in enumerate(response.trajectory.points):
            total_time = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
            times.append(total_time)
            joint_count = len(response.trajectory.joint_names)
            for joint_idx, position in enumerate(point.positions):
                pos_series[joint_idx].append(position)
            if len(point.velocities) == joint_count:
                for joint_idx in range(joint_count):
                    vel_series[joint_idx].append(point.velocities[joint_idx])
            if len(point.accelerations) == joint_count:
                for joint_idx in range(joint_count):
                    acc_series[joint_idx].append(point.accelerations[joint_idx])
            node.get_logger().info(
                f'Point {i}: t={total_time:.3f}s positions={list(point.positions)}'
            )

        if plt is None:
            node.get_logger().warning('matplotlib not available; skipping plot')
        else:
            pos_series_np = np.array(pos_series) if np is not None else None
            times_np = np.array(times) if np is not None else None

            # Fallback derivatives if planner did not return them
            if vel_series and not vel_series[0]:
                if np is None:
                    node.get_logger().warning('NumPy not available; cannot estimate velocity profile')
                else:
                    vel_series = np.gradient(pos_series_np, times_np, axis=1).tolist()
            if acc_series and not acc_series[0]:
                if np is None:
                    node.get_logger().warning('NumPy not available; cannot estimate acceleration profile')
                else:
                    acc_series = np.gradient(np.array(vel_series), times_np, axis=1).tolist()

            original_time = list(range(len(test_waypoints)))
            for joint_idx, joint_name in enumerate(response.trajectory.joint_names):
                plt.figure(1)
                plt.plot(times, pos_series[joint_idx], '.-', label=f'smoothed {joint_name}')
                plt.plot(
                    original_time,
                    [test_waypoints[i][joint_idx] for i in range(len(test_waypoints))],
                    'o--',
                    label=f'raw {joint_name}',
                    alpha=0.6,
                )

                if vel_series and vel_series[joint_idx]:
                    plt.figure(2)
                    plt.plot(times, vel_series[joint_idx], label=joint_name)

                if acc_series and acc_series[joint_idx]:
                    plt.figure(3)
                    plt.plot(times, acc_series[joint_idx], label=joint_name)

            plt.figure(1)
            plt.title('Raw vs Smoothed Joint Trajectory')
            plt.xlabel('time [s]')
            plt.ylabel('joint position [rad]')
            plt.legend()
            plt.grid(True)

            if vel_series and vel_series[0]:
                plt.figure(2)
                plt.title('Joint Velocity Profiles')
                plt.xlabel('time [s]')
                plt.ylabel('joint velocity [rad/s]')
                plt.legend()
                plt.grid(True)

            if acc_series and acc_series[0]:
                plt.figure(3)
                plt.title('Joint Acceleration Profiles')
                plt.xlabel('time [s]')
                plt.ylabel('joint acceleration [rad/s²]')
                plt.legend()
                plt.grid(True)

            for fig_id in plt.get_fignums():
                plt.figure(fig_id)
                plt.tight_layout()
            plt.show()
    else:
        node.get_logger().error(f'FAILED: {response.message}')
    
    rclpy.shutdown()

if __name__ == '__main__':
    test_smoother()
