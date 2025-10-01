// Copyright (c) 2024

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>

#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/collision_distance_field/collision_detector_allocator_hybrid.h>

#include <chomp_motion_planner/chomp_optimizer.h>
#include <chomp_motion_planner/chomp_parameters.h>
#include <chomp_motion_planner/chomp_trajectory.h>

#include "trajectory_smoother_service/srv/smooth_trajectory.hpp"

namespace
{
constexpr double kScalingMin = 1e-3;

inline double toSeconds(const builtin_interfaces::msg::Duration& d)
{
  return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
}

inline builtin_interfaces::msg::Duration toDuration(double t)
{
  builtin_interfaces::msg::Duration out;
  if (t < 0.0)
  {
    t = 0.0;
  }
  const double s = std::floor(t);
  out.sec = static_cast<int32_t>(s);
  out.nanosec = static_cast<uint32_t>(std::round((t - s) * 1e9));
  return out;
}

trajectory_msgs::msg::JointTrajectory resampleQuintic(const trajectory_msgs::msg::JointTrajectory& in, double dt)
{
  trajectory_msgs::msg::JointTrajectory out;
  out.joint_names = in.joint_names;
  out.header = in.header;

  if (in.points.size() < 2 || dt <= 0.0)
  {
    out.points = in.points;
    return out;
  }

  const std::size_t joint_count = in.joint_names.size();
  out.points.reserve(in.points.size());
  out.points.push_back(in.points.front());

  auto access = [&](const std::vector<double>& data, std::size_t idx, double fallback) {
    return (data.size() == joint_count) ? data[idx] : fallback;
  };

  for (std::size_t seg = 0; seg + 1 < in.points.size(); ++seg)
  {
    const auto& p0 = in.points[seg];
    const auto& p1 = in.points[seg + 1];

    const double t0 = toSeconds(p0.time_from_start);
    const double t1 = toSeconds(p1.time_from_start);
    double T = t1 - t0;
    if (T <= 1e-9)
    {
      T = 1e-9;
    }

    const int steps = std::max(1, static_cast<int>(std::floor(T / dt)));

    for (int i = 1; i <= steps; ++i)
    {
      const double tau = static_cast<double>(i) * dt;
      if (tau >= T - 1e-9)
      {
        continue;
      }

      const double s = tau / T;
      const double s2 = s * s;
      const double s3 = s2 * s;
      const double s4 = s3 * s;
      const double s5 = s4 * s;

      const double h00 = 1 - 10 * s3 + 15 * s4 - 6 * s5;
      const double h01 = 10 * s3 - 15 * s4 + 6 * s5;
      const double h10 = s - 6 * s3 + 8 * s4 - 3 * s5;
      const double h11 = -4 * s3 + 7 * s4 - 3 * s5;
      const double h20 = 0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5;
      const double h21 = 0.5 * s3 - s4 + 0.5 * s5;

      const double dh00 = -30 * s2 + 60 * s3 - 30 * s4;
      const double dh01 = 30 * s2 - 60 * s3 + 30 * s4;
      const double dh10 = 1.0 - 18 * s2 + 32 * s3 - 15 * s4;
      const double dh11 = -12 * s2 + 28 * s3 - 15 * s4;
      const double dh20 = s - 4.5 * s2 + 6 * s3 - 2.5 * s4;
      const double dh21 = 1.5 * s2 - 4 * s3 + 2.5 * s4;

      const double d2h00 = -60 * s + 180 * s2 - 120 * s3;
      const double d2h01 = 60 * s - 180 * s2 + 120 * s3;
      const double d2h10 = -36 * s + 96 * s2 - 60 * s3;
      const double d2h11 = -24 * s + 84 * s2 - 60 * s3;
      const double d2h20 = 1.0 - 9 * s + 18 * s2 - 10 * s3;
      const double d2h21 = 3 * s - 12 * s2 + 10 * s3;

      trajectory_msgs::msg::JointTrajectoryPoint pt;
      pt.positions.resize(joint_count);
      pt.velocities.resize(joint_count);
      pt.accelerations.resize(joint_count);

      for (std::size_t j = 0; j < joint_count; ++j)
      {
        const double q0 = access(p0.positions, j, 0.0);
        const double q1 = access(p1.positions, j, 0.0);
        const double v0 = access(p0.velocities, j, 0.0);
        const double v1 = access(p1.velocities, j, 0.0);
        const double a0 = access(p0.accelerations, j, 0.0);
        const double a1 = access(p1.accelerations, j, 0.0);

        const double q = h00 * q0 + h01 * q1 + h10 * (T * v0) + h11 * (T * v1) + h20 * (T * T * a0) +
                         h21 * (T * T * a1);
        const double dqds = dh00 * q0 + dh01 * q1 + dh10 * (T * v0) + dh11 * (T * v1) + dh20 * (T * T * a0) +
                            dh21 * (T * T * a1);
        const double d2qds2 = d2h00 * q0 + d2h01 * q1 + d2h10 * (T * v0) + d2h11 * (T * v1) +
                              d2h20 * (T * T * a0) + d2h21 * (T * T * a1);

        pt.positions[j] = q;
        pt.velocities[j] = dqds / T;
        pt.accelerations[j] = d2qds2 / (T * T);
      }

      pt.time_from_start = toDuration(t0 + tau);
      out.points.emplace_back(std::move(pt));
    }
  }

  out.points.push_back(in.points.back());
  return out;
}
}  // namespace

class TrajectorySmoother : public rclcpp::Node
{
public:
  TrajectorySmoother() : rclcpp::Node("trajectory_smoother_node")
  {
    declareParameters();

    smooth_trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        output_topic_, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    init_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&TrajectorySmoother::initializeMoveIt, this));
  }

private:
  void declareParameters()
  {
    planning_group_name_ = this->declare_parameter<std::string>("planning_group", "ur_manipulator");
    output_topic_ = this->declare_parameter<std::string>("smoothed_topic", "smooth_trajectory");
    publish_on_success_ = this->declare_parameter<bool>("publish_on_success", true);
    initial_discretization_ = this->declare_parameter<double>("initial_discretization", 0.05);
    if (initial_discretization_ <= 0.0)
    {
      RCLCPP_WARN(get_logger(), "initial_discretization must be > 0. Using 0.05 s.");
      initial_discretization_ = 0.05;
    }

    default_velocity_scaling_ = this->declare_parameter<double>("default_velocity_scaling", 0.5);
    default_acceleration_scaling_ = this->declare_parameter<double>("default_acceleration_scaling", 0.5);

    totg_path_tolerance_ = this->declare_parameter<double>("time_parameterization.path_tolerance", 0.1);
    totg_resample_dt_ = this->declare_parameter<double>("time_parameterization.resample_dt", 0.1);
    totg_min_angle_change_ = this->declare_parameter<double>("time_parameterization.min_angle_change", 0.001);
  }

  void initializeMoveIt()
  {
    if (initialized_)
    {
      return;
    }

    try
    {
      if (!robot_model_loader_)
      {
        robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(shared_from_this(),
                                                                                    "robot_description");
      }
    }
    catch (const std::exception& ex)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "RobotModelLoader not ready yet: %s", ex.what());
      return;
    }

    robot_model_ = robot_model_loader_->getModel();
    if (!robot_model_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for robot_description to become available");
      return;
    }

    joint_model_group_ = robot_model_->getJointModelGroup(planning_group_name_);
    if (!joint_model_group_)
    {
      joint_model_group_ = robot_model_->getJointModelGroup("manipulator");
    }

    if (!joint_model_group_)
    {
      RCLCPP_ERROR(get_logger(), "Failed to resolve joint model group '%s'", planning_group_name_.c_str());
      return;
    }

    try
    {
      planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
      planning_scene_->allocateCollisionDetector(collision_detection::CollisionDetectorAllocatorHybrid::create());
      planning_scene_->getCurrentStateNonConst().setToDefaultValues();
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(get_logger(), "Failed to initialise planning scene: %s", ex.what());
      return;
    }

    chomp_parameters_ = std::make_shared<chomp::ChompParameters>();
    configureChompParameters();

    service_ = this->create_service<trajectory_smoother_service::srv::SmoothTrajectory>(
        "smooth_trajectory",
        std::bind(&TrajectorySmoother::smoothTrajectoryCallback, this, std::placeholders::_1, std::placeholders::_2));

    initialized_ = true;
    init_timer_->cancel();

    RCLCPP_INFO(get_logger(), "Trajectory smoother ready (CHOMP + TOTG) for group '%s'", joint_model_group_->getName().c_str());
  }

  void configureChompParameters()
  {
    if (!chomp_parameters_)
    {
      return;
    }

    chomp_parameters_->planning_time_limit_ = this->declare_parameter<double>(
        "chomp.planning_time_limit", chomp_parameters_->planning_time_limit_);
    chomp_parameters_->max_iterations_ = static_cast<int>(this->declare_parameter<int64_t>(
        "chomp.max_iterations", chomp_parameters_->max_iterations_));
    chomp_parameters_->max_iterations_after_collision_free_ = static_cast<int>(this->declare_parameter<int64_t>(
        "chomp.max_iterations_after_collision_free", chomp_parameters_->max_iterations_after_collision_free_));
    chomp_parameters_->smoothness_cost_weight_ = this->declare_parameter<double>(
        "chomp.smoothness_cost_weight", chomp_parameters_->smoothness_cost_weight_);
    chomp_parameters_->obstacle_cost_weight_ = this->declare_parameter<double>(
        "chomp.obstacle_cost_weight", chomp_parameters_->obstacle_cost_weight_);
    chomp_parameters_->learning_rate_ = this->declare_parameter<double>(
        "chomp.learning_rate", chomp_parameters_->learning_rate_);
    chomp_parameters_->smoothness_cost_velocity_ = this->declare_parameter<double>(
        "chomp.smoothness_cost_velocity", chomp_parameters_->smoothness_cost_velocity_);
    chomp_parameters_->smoothness_cost_acceleration_ = this->declare_parameter<double>(
        "chomp.smoothness_cost_acceleration", chomp_parameters_->smoothness_cost_acceleration_);
    chomp_parameters_->smoothness_cost_jerk_ = this->declare_parameter<double>(
        "chomp.smoothness_cost_jerk", chomp_parameters_->smoothness_cost_jerk_);
    chomp_parameters_->use_stochastic_descent_ = this->declare_parameter<bool>(
        "chomp.use_stochastic_descent", chomp_parameters_->use_stochastic_descent_);
    chomp_parameters_->ridge_factor_ = this->declare_parameter<double>(
        "chomp.ridge_factor", chomp_parameters_->ridge_factor_);
    chomp_parameters_->use_pseudo_inverse_ = this->declare_parameter<bool>(
        "chomp.use_pseudo_inverse", chomp_parameters_->use_pseudo_inverse_);
    chomp_parameters_->pseudo_inverse_ridge_factor_ = this->declare_parameter<double>(
        "chomp.pseudo_inverse_ridge_factor", chomp_parameters_->pseudo_inverse_ridge_factor_);
    chomp_parameters_->joint_update_limit_ = this->declare_parameter<double>(
        "chomp.joint_update_limit", chomp_parameters_->joint_update_limit_);
    chomp_parameters_->min_clearance_ = this->declare_parameter<double>(
        "chomp.min_clearance", chomp_parameters_->min_clearance_);
    chomp_parameters_->collision_threshold_ = this->declare_parameter<double>(
        "chomp.collision_threshold", chomp_parameters_->collision_threshold_);
    chomp_parameters_->filter_mode_ = this->declare_parameter<bool>(
        "chomp.filter_mode", chomp_parameters_->filter_mode_);
    chomp_parameters_->enable_failure_recovery_ = this->declare_parameter<bool>(
        "chomp.enable_failure_recovery", chomp_parameters_->enable_failure_recovery_);
    chomp_parameters_->max_recovery_attempts_ = static_cast<int>(this->declare_parameter<int64_t>(
        "chomp.max_recovery_attempts", chomp_parameters_->max_recovery_attempts_));

    const std::string init_method = this->declare_parameter<std::string>(
        "chomp.trajectory_initialization_method", chomp_parameters_->trajectory_initialization_method_);
    if (!chomp_parameters_->setTrajectoryInitializationMethod(init_method))
    {
      RCLCPP_WARN(get_logger(), "Invalid CHOMP initialisation method '%s'. Keeping '%s'.",
                  init_method.c_str(), chomp_parameters_->trajectory_initialization_method_.c_str());
    }
  }

  double clampScalingFactor(double requested, double fallback) const
  {
    double value = requested > 0.0 ? requested : fallback;
    value = std::clamp(value, kScalingMin, 1.0);
    return value;
  }

  void smoothTrajectoryCallback(
      const std::shared_ptr<trajectory_smoother_service::srv::SmoothTrajectory::Request>& request,
      std::shared_ptr<trajectory_smoother_service::srv::SmoothTrajectory::Response> response)
  {
    if (!initialized_)
    {
      response->success = false;
      response->message = "Trajectory smoother is not initialised yet";
      RCLCPP_WARN(get_logger(), "Received smoothing request before initialisation complete");
      return;
    }

    try
    {
      const std::vector<std::string>& joint_names = joint_model_group_->getActiveJointModelNames();
      const std::size_t expected_dimension = joint_names.size();

      if (request->waypoints.size() < 2)
      {
        response->success = false;
        response->message = "At least two waypoints are required";
        RCLCPP_WARN(get_logger(), "Received %zu waypoints; need at least 2", request->waypoints.size());
        return;
      }

      trajectory_msgs::msg::JointTrajectory initial_joint_trajectory;
      initial_joint_trajectory.header.stamp = this->now();
      initial_joint_trajectory.header.frame_id = planning_scene_->getPlanningFrame();
      initial_joint_trajectory.joint_names = joint_names;

      robot_trajectory::RobotTrajectory input_trajectory(robot_model_, joint_model_group_);

      const double discretization = std::max(initial_discretization_, 1e-3);

      for (std::size_t index = 0; index < request->waypoints.size(); ++index)
      {
        const auto& waypoint = request->waypoints[index];
        if (waypoint.positions.size() != expected_dimension)
        {
          response->success = false;
          response->message = "Waypoint dimension does not match planning group";
          RCLCPP_ERROR(get_logger(), "Waypoint %zu has %zu elements, expected %zu", index, waypoint.positions.size(),
                       expected_dimension);
          return;
        }

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = waypoint.positions;
        if (waypoint.velocities.size() == expected_dimension)
        {
          point.velocities = waypoint.velocities;
        }
        if (waypoint.accelerations.size() == expected_dimension)
        {
          point.accelerations = waypoint.accelerations;
        }
        const double accumulated_time = static_cast<double>(index) * discretization;
        builtin_interfaces::msg::Duration duration_msg;
        duration_msg.sec = static_cast<int32_t>(std::floor(accumulated_time));
        duration_msg.nanosec = static_cast<uint32_t>(
            std::round((accumulated_time - static_cast<double>(duration_msg.sec)) * 1e9));
        point.time_from_start = duration_msg;
        initial_joint_trajectory.points.push_back(point);

        moveit::core::RobotState state(robot_model_);
        state.setJointGroupPositions(joint_model_group_, waypoint.positions);
        state.update();
        input_trajectory.addSuffixWayPoint(state, index == 0 ? 0.0 : discretization);
      }

      moveit::core::RobotState start_state(robot_model_);
      start_state.setJointGroupPositions(joint_model_group_, initial_joint_trajectory.points.front().positions);
      start_state.update();
      planning_scene_->setCurrentState(start_state);

      chomp::ChompTrajectory chomp_trajectory(robot_model_, request->waypoints.size(), discretization,
                                              joint_model_group_->getName());
      const bool chomp_seed_ok = chomp_trajectory.fillInFromTrajectory(input_trajectory);
      if (!chomp_seed_ok)
      {
        RCLCPP_WARN(get_logger(), "Failed to initialise CHOMP trajectory from supplied waypoints; using raw trajectory");
      }

      bool optimisation_ok = false;

      try
      {
        if (chomp_seed_ok)
        {
          chomp::ChompOptimizer optimizer(&chomp_trajectory, planning_scene_, joint_model_group_->getName(),
                                          chomp_parameters_.get(), start_state);
          if (!optimizer.isInitialized())
          {
            RCLCPP_WARN(get_logger(), "CHOMP optimizer failed to initialise; falling back to raw trajectory");
          }
          else
          {
            optimisation_ok = optimizer.optimize();
            if (!optimisation_ok)
            {
              RCLCPP_WARN(get_logger(), "CHOMP optimisation did not converge; using input trajectory");
            }
            else if (!optimizer.isCollisionFree())
            {
              RCLCPP_WARN(get_logger(), "CHOMP result in collision; using input trajectory instead");
              optimisation_ok = false;
            }
          }
        }
      }
      catch (const std::exception& ex)
      {
        RCLCPP_WARN(get_logger(), "CHOMP optimisation threw an exception: %s", ex.what());
        optimisation_ok = false;
      }

      robot_trajectory::RobotTrajectory chomp_result(robot_model_, joint_model_group_);
      if (optimisation_ok)
      {
        const std::size_t num_points = chomp_trajectory.getNumPoints();
        const double chomp_dt = chomp_trajectory.getDiscretization() > 0.0 ? chomp_trajectory.getDiscretization()
                                                                           : discretization;
        std::vector<double> positions(expected_dimension, 0.0);

        for (std::size_t traj_index = 0; traj_index < num_points; ++traj_index)
        {
          for (std::size_t joint_index = 0; joint_index < expected_dimension; ++joint_index)
          {
            positions[joint_index] = chomp_trajectory(traj_index, joint_index);
          }

          moveit::core::RobotState state(robot_model_);
          state.setJointGroupPositions(joint_model_group_, positions);
          state.update();
          chomp_result.addSuffixWayPoint(state, traj_index == 0 ? 0.0 : chomp_dt);
        }
      }

      robot_trajectory::RobotTrajectory* working_trajectory = optimisation_ok ? &chomp_result : &input_trajectory;

      const double velocity_scaling = clampScalingFactor(request->max_velocity_scaling_factor, default_velocity_scaling_);
      const double acceleration_scaling =
          clampScalingFactor(request->max_acceleration_scaling_factor, default_acceleration_scaling_);

      const double totg_dt =
          (request->totg_resample_dt > 0.0) ? request->totg_resample_dt : totg_resample_dt_;
      trajectory_processing::TimeOptimalTrajectoryGeneration totg(totg_path_tolerance_, totg_dt,
                                                                 totg_min_angle_change_);
      bool timing_ok = totg.computeTimeStamps(*working_trajectory, velocity_scaling, acceleration_scaling);

      if (!timing_ok)
      {
        RCLCPP_WARN(get_logger(), "Time-optimal parameterisation failed; falling back to iterative parabolic timing");
        trajectory_processing::IterativeParabolicTimeParameterization iptp;
        timing_ok = iptp.computeTimeStamps(*working_trajectory, velocity_scaling, acceleration_scaling);
      }

      if (!timing_ok)
      {
        RCLCPP_WARN(get_logger(), "Iterative parabolic parameterisation failed; applying simple timestamps");
        applySimpleTimeStamps(*working_trajectory, discretization);
      }

      moveit_msgs::msg::RobotTrajectory robot_traj_msg;
      working_trajectory->getRobotTrajectoryMsg(robot_traj_msg);
      robot_traj_msg.joint_trajectory.header.frame_id = planning_scene_->getPlanningFrame();
      robot_traj_msg.joint_trajectory.header.stamp = this->now();

      double hermite_dt = (request->hermite_resample_dt > 0.0) ? request->hermite_resample_dt : totg_dt;
      hermite_dt = std::clamp(hermite_dt, 1e-4, 1.0);

      response->trajectory = resampleQuintic(robot_traj_msg.joint_trajectory, hermite_dt);
      response->success = true;
      response->message = optimisation_ok ? "CHOMP optimisation success" : "Returned time-parameterised input trajectory";

      if (publish_on_success_)
      {
        smooth_trajectory_pub_->publish(response->trajectory);
      }

      RCLCPP_INFO(get_logger(),
                  "Provided %s trajectory with %zu waypoints (vel_scale=%.3f, acc_scale=%.3f, totg_dt=%.4f, hermite_dt=%.4f)",
                  optimisation_ok ? "smoothed" : "original", response->trajectory.points.size(), velocity_scaling,
                  acceleration_scaling, totg_dt, hermite_dt);
    }
    catch (const std::exception& ex)
    {
      response->success = false;
      response->message = ex.what();
      RCLCPP_ERROR(get_logger(), "Failed to provide smoothed trajectory: %s", ex.what());
    }
  }

  void applySimpleTimeStamps(robot_trajectory::RobotTrajectory& trajectory, double step) const
  {
    const double timestep = std::max(step, initial_discretization_);
    for (std::size_t idx = 0; idx < trajectory.getWayPointCount(); ++idx)
    {
      trajectory.setWayPointDurationFromPrevious(idx, idx == 0 ? 0.0 : timestep);
    }
  }

  // MoveIt resources
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;
  planning_scene::PlanningScenePtr planning_scene_;
  const moveit::core::JointModelGroup* joint_model_group_{ nullptr };
  std::shared_ptr<chomp::ChompParameters> chomp_parameters_;

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr smooth_trajectory_pub_;
  rclcpp::Service<trajectory_smoother_service::srv::SmoothTrajectory>::SharedPtr service_;

  std::string planning_group_name_;
  std::string output_topic_;
  bool publish_on_success_{ true };
  double initial_discretization_{ 0.05 };
  double default_velocity_scaling_{ 0.5 };
  double default_acceleration_scaling_{ 0.5 };
  double totg_path_tolerance_{ 0.1 };
  double totg_resample_dt_{ 0.1 };
  double totg_min_angle_change_{ 0.001 };
  bool initialized_{ false };
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectorySmoother>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
