// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

#include <gtest/gtest.h>
#include <memory>

#include "easynav_nav2_controller/CostmapController.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <tf2_ros/static_transform_broadcaster.h>


class CostmapControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  nav_msgs::msg::OccupancyGrid createOccupancyGrid(
    int width, int height, double resolution,
    double origin_x, double origin_y)
  {
    nav_msgs::msg::MapMetaData meta_data;
    meta_data.resolution = resolution;
    meta_data.width = width;
    meta_data.height = height;
    meta_data.origin.position.x = origin_x;
    meta_data.origin.position.y = origin_y;
    meta_data.origin.position.z = 0.0;
    meta_data.origin.orientation.w = 1.0;
    meta_data.origin.orientation.x = 0.0;
    meta_data.origin.orientation.y = 0.0;
    meta_data.origin.orientation.z = 0.0;
    meta_data.map_load_time = rclcpp::Time(0);

    return createOccupancyGrid(meta_data);
  }

  nav_msgs::msg::OccupancyGrid createOccupancyGrid(const nav_msgs::msg::MapMetaData & meta_data)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.info = meta_data;
    grid.data.resize(meta_data.width * meta_data.height, -1);
    grid.header.frame_id = "map";
    grid.header.stamp = rclcpp::Time(0);
    return grid;
  }

  nav_msgs::msg::Odometry createInitialPose()
  {
    nav_msgs::msg::Odometry initial_pose;
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = rclcpp::Time(0);
    initial_pose.pose.pose.position.x = 1.0;
    initial_pose.pose.pose.position.y = 1.0;
    initial_pose.pose.pose.position.z = 0.0;
    initial_pose.pose.pose.orientation.w = 1.0;
    initial_pose.pose.pose.orientation.x = 0.0;
    initial_pose.pose.pose.orientation.y = 0.0;
    initial_pose.pose.pose.orientation.z = 0.0;
    initial_pose.pose.covariance.fill(0.0);
    initial_pose.twist.twist.linear.x = 0.00000001;
    initial_pose.twist.twist.linear.y = 0.00000001;
    initial_pose.twist.twist.angular.z = 0.000000001;
    initial_pose.twist.covariance.fill(0.0);
    initial_pose.child_frame_id = "base_link";

    return initial_pose;
  }

  nav_msgs::msg::Path createPath()
  {
    std::vector<double> points_x = {
      -2.0603694915771484,
      -1.603752613067627,
      -0.9857032895088196,
      -0.31749987602233887,
      0.23710575795173645,
      0.6451051831245422,
      1.2736501693725586,
      1.920559287071228
    };

    std::vector<double> points_y = {
      0.0427580326795578,
      0.36432763934135437,
      0.5359398722648621,
      0.6033092737197876,
      0.5728853940963745,
      0.45112180709838867,
      0.16400346159934998,
      -0.2185245156288147
    };

    nav_msgs::msg::Path path;
    path.header.stamp = rclcpp::Clock().now();
    path.header.frame_id = "map";

    for (size_t i = 0; i < points_x.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = points_x[i];
      pose.pose.position.y = points_y[i];
      pose.pose.position.z = 0.0;
      pose.pose.orientation.w = 1.0;

      path.poses.push_back(pose);
    }

    return path;
  }
};

TEST_F(CostmapControllerTest, Initialize)
{
  rclcpp::NodeOptions options;

  // Plugins base
  options.append_parameter_override("controller_frequency", 20.0);
  options.append_parameter_override("test.progress_checker_plugin",
    std::string("progress_checker"));
  options.append_parameter_override("test.goal_checker_plugin",
    std::string("general_goal_checker"));
  options.append_parameter_override("test.controller_plugin", std::string("FollowPath"));

  // Plugin definitions
  options.append_parameter_override("test.progress_checker.plugin",
    std::string("nav2_controller::SimpleProgressChecker"));
  options.append_parameter_override("test.progress_checker.required_movement_radius", 0.5);
  options.append_parameter_override("test.progress_checker.movement_time_allowance", 10.0);

  options.append_parameter_override("test.general_goal_checker.plugin",
    std::string("nav2_controller::SimpleGoalChecker"));
  options.append_parameter_override("test.general_goal_checker.stateful", true);
  options.append_parameter_override("test.general_goal_checker.xy_goal_tolerance", 0.25);
  options.append_parameter_override("test.general_goal_checker.yaw_goal_tolerance", 0.25);

  options.append_parameter_override("test.FollowPath.plugin",
    std::string("nav2_mppi_controller::MPPIController"));
  options.append_parameter_override("test.FollowPath.time_steps", 56);
  options.append_parameter_override("test.FollowPath.model_dt", 0.05);
  options.append_parameter_override("test.FollowPath.batch_size", 2000);
  options.append_parameter_override("test.FollowPath.ax_max", 3.0);
  options.append_parameter_override("test.FollowPath.ax_min", -3.0);
  options.append_parameter_override("test.FollowPath.ay_max", 3.0);
  options.append_parameter_override("test.FollowPath.ay_min", -3.0);
  options.append_parameter_override("test.FollowPath.az_max", 3.5);
  options.append_parameter_override("test.FollowPath.vx_std", 0.8);
  options.append_parameter_override("test.FollowPath.vy_std", 0.8);
  options.append_parameter_override("test.FollowPath.wz_std", 0.8);
  options.append_parameter_override("test.FollowPath.vx_max", 0.8);
  options.append_parameter_override("test.FollowPath.vx_min", -0.35);
  options.append_parameter_override("test.FollowPath.vy_max", 0.8);
  options.append_parameter_override("test.FollowPath.wz_max", 1.9);
  options.append_parameter_override("test.FollowPath.iteration_count", 1);
  options.append_parameter_override("test.FollowPath.prune_distance", 0.5);
  options.append_parameter_override("test.FollowPath.transform_tolerance", 0.1);
  options.append_parameter_override("test.FollowPath.temperature", 0.3);
  options.append_parameter_override("test.FollowPath.gamma", 0.015);
  options.append_parameter_override("test.FollowPath.motion_model", std::string("DiffDrive"));
  options.append_parameter_override("test.FollowPath.visualize", true);
  options.append_parameter_override("test.FollowPath.publish_optimal_trajectory", true);
  options.append_parameter_override("test.FollowPath.regenerate_noises", true);

  // TrajectoryVisualizer
  options.append_parameter_override("test.FollowPath.TrajectoryVisualizer.trajectory_step", 5);
  options.append_parameter_override("test.FollowPath.TrajectoryVisualizer.time_step", 3);

  // AckermannConstraints
  options.append_parameter_override("test.FollowPath.AckermannConstraints.min_turning_r", 0.2);

  // Critics
  options.append_parameter_override("test.FollowPath.critics", std::vector<std::string>{
    "ConstraintCritic",
    "CostCritic",
    "GoalCritic",
    "GoalAngleCritic",
    "PathAlignCritic",
    "PathFollowCritic",
    "PathAngleCritic",
    "PreferForwardCritic"
  });

  // ConstraintCritic
  options.append_parameter_override("test.FollowPath.ConstraintCritic.enabled", true);
  options.append_parameter_override("test.FollowPath.ConstraintCritic.cost_power", 1);
  options.append_parameter_override("test.FollowPath.ConstraintCritic.cost_weight", 4.0);

  // GoalCritic
  options.append_parameter_override("test.FollowPath.GoalCritic.enabled", true);
  options.append_parameter_override("test.FollowPath.GoalCritic.cost_power", 1);
  options.append_parameter_override("test.FollowPath.GoalCritic.cost_weight", 5.0);
  options.append_parameter_override("test.FollowPath.GoalCritic.threshold_to_consider", 1.4);

  // GoalAngleCritic
  options.append_parameter_override("test.FollowPath.GoalAngleCritic.enabled", true);
  options.append_parameter_override("test.FollowPath.GoalAngleCritic.cost_power", 1);
  options.append_parameter_override("test.FollowPath.GoalAngleCritic.cost_weight", 3.0);
  options.append_parameter_override("test.FollowPath.GoalAngleCritic.threshold_to_consider", 0.5);

  // PreferForwardCritic
  options.append_parameter_override("test.FollowPath.PreferForwardCritic.enabled", true);
  options.append_parameter_override("test.FollowPath.PreferForwardCritic.cost_power", 1);
  options.append_parameter_override("test.FollowPath.PreferForwardCritic.cost_weight", 5.0);
  options.append_parameter_override("test.FollowPath.PreferForwardCritic.threshold_to_consider",
    0.5);

  // CostCritic
  options.append_parameter_override("test.FollowPath.CostCritic.enabled", true);
  options.append_parameter_override("test.FollowPath.CostCritic.cost_power", 1);
  options.append_parameter_override("test.FollowPath.CostCritic.cost_weight", 3.81);
  options.append_parameter_override("test.FollowPath.CostCritic.near_collision_cost", 253);
  options.append_parameter_override("test.FollowPath.CostCritic.critical_cost", 300.0);
  options.append_parameter_override("test.FollowPath.CostCritic.consider_footprint", false);
  options.append_parameter_override("test.FollowPath.CostCritic.collision_cost", 1000000.0);
  options.append_parameter_override("test.FollowPath.CostCritic.near_goal_distance", 1.0);
  options.append_parameter_override("test.FollowPath.CostCritic.trajectory_point_step", 2);

  // PathAlignCritic
  options.append_parameter_override("test.FollowPath.PathAlignCritic.enabled", true);
  options.append_parameter_override("test.FollowPath.PathAlignCritic.cost_power", 1);
  options.append_parameter_override("test.FollowPath.PathAlignCritic.cost_weight", 14.0);
  options.append_parameter_override("test.FollowPath.PathAlignCritic.max_path_occupancy_ratio",
    0.05);
  options.append_parameter_override("test.FollowPath.PathAlignCritic.trajectory_point_step", 4);
  options.append_parameter_override("test.FollowPath.PathAlignCritic.threshold_to_consider", 0.5);
  options.append_parameter_override("test.FollowPath.PathAlignCritic.offset_from_furthest", 20);
  options.append_parameter_override("test.FollowPath.PathAlignCritic.use_path_orientations", false);

  // PathFollowCritic
  options.append_parameter_override("test.FollowPath.PathFollowCritic.enabled", true);
  options.append_parameter_override("test.FollowPath.PathFollowCritic.cost_power", 1);
  options.append_parameter_override("test.FollowPath.PathFollowCritic.cost_weight", 5.0);
  options.append_parameter_override("test.FollowPath.PathFollowCritic.offset_from_furthest", 5);
  options.append_parameter_override("test.FollowPath.PathFollowCritic.threshold_to_consider", 1.4);

  // PathAngleCritic
  options.append_parameter_override("test.FollowPath.PathAngleCritic.enabled", true);
  options.append_parameter_override("test.FollowPath.PathAngleCritic.cost_power", 1);
  options.append_parameter_override("test.FollowPath.PathAngleCritic.cost_weight", 2.0);
  options.append_parameter_override("test.FollowPath.PathAngleCritic.offset_from_furthest", 4);
  options.append_parameter_override("test.FollowPath.PathAngleCritic.threshold_to_consider", 0.5);
  options.append_parameter_override("test.FollowPath.PathAngleCritic.max_angle_to_furthest", 1.0);
  options.append_parameter_override("test.FollowPath.PathAngleCritic.mode", 0);


  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("costmap_planner_node", options);

  auto controller = std::make_shared<easynav::CostmapController>();

  auto result = controller->initialize(node, "test");
  EXPECT_TRUE(result.has_value());

  auto map = createOccupancyGrid(300, 300, 0.05, 0.0, 0.0);
  auto publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  publisher->on_activate();
  publisher->publish(map);

  easynav::NavState navstate;

  navstate.odom = createInitialPose();

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = node->now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "base_link";
  tf_msg.transform.translation.x = 1.0;
  tf_msg.transform.translation.y = 1.0;
  tf_msg.transform.translation.z = 0.0;
  tf_msg.transform.rotation.x = 0.0;
  tf_msg.transform.rotation.y = 0.0;
  tf_msg.transform.rotation.z = 0.0;
  tf_msg.transform.rotation.w = 1.0;

  auto tf_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
  tf_broadcaster->sendTransform(tf_msg);

  rclcpp::sleep_for(std::chrono::milliseconds(100));

  auto twist = controller->get_cmd_vel();
  EXPECT_EQ(twist.twist.linear.x, 0.0);

  controller->update_rt(navstate);

  navstate.path = createPath();

  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  controller->update_rt(navstate);

  twist = controller->get_cmd_vel();
  EXPECT_NE(twist.twist.linear.x, 0.0);
  EXPECT_EQ(twist.header.frame_id, "map");
}
