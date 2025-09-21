#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include "trajectory_smoother_service/srv/smooth_trajectory.hpp"

class SimpleTrajectorySmoother : public rclcpp::Node
{
public:
    SimpleTrajectorySmoother() : Node("trajectory_smoother_node")
    {
        // 서비스 서버 생성
        service_ = this->create_service<trajectory_smoother_service::srv::SmoothTrajectory>(
            "smooth_trajectory",
            std::bind(&SimpleTrajectorySmoother::smoothTrajectoryCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        RCLCPP_INFO(this->get_logger(), "Simple Trajectory Smoother Service Ready!");
    }

private:
    void smoothTrajectoryCallback(
        const std::shared_ptr<trajectory_smoother_service::srv::SmoothTrajectory::Request> request,
        std::shared_ptr<trajectory_smoother_service::srv::SmoothTrajectory::Response> response)
    {
        try {
            // 일단 간단한 시간 할당만 수행 (테스트용)
            response->trajectory.joint_names = {
                "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
            };
            
            for (size_t i = 0; i < request->waypoints.size(); ++i) {
                trajectory_msgs::msg::JointTrajectoryPoint point;
                point.positions = request->waypoints[i].positions;
                point.time_from_start.sec = static_cast<int32_t>(i + 1);
                point.time_from_start.nanosec = 0;
                
                response->trajectory.points.push_back(point);
            }
            
            response->success = true;
            response->message = "Simple trajectory created successfully";
            
            RCLCPP_INFO(this->get_logger(), 
                       "Created simple trajectory with %zu waypoints", 
                       request->waypoints.size());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
            response->success = false;
            response->message = e.what();
        }
    }

    rclcpp::Service<trajectory_smoother_service::srv::SmoothTrajectory>::SharedPtr service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTrajectorySmoother>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}