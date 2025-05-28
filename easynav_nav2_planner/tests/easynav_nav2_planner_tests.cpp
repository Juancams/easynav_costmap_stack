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

#include "easynav_nav2_planner/CostmapPlanner.hpp"
#include "easynav_nav2_costmap_maps_manager/Costmap.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <tf2_ros/static_transform_broadcaster.h>


class CostmapPlannerTest : public ::testing::Test
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

    return initial_pose;
  }

  nav_msgs::msg::Goals createGoal()
  {
    nav_msgs::msg::Goals goal;
    goal.header.frame_id = "map";
    goal.header.stamp = rclcpp::Time(0);
    goal.goals.resize(1);
    goal.goals[0].header.frame_id = "map";
    goal.goals[0].header.stamp = rclcpp::Time(0);
    goal.goals[0].pose.position.x = 2.0;
    goal.goals[0].pose.position.y = 1.0;
    goal.goals[0].pose.position.z = 0.0;
    goal.goals[0].pose.orientation.w = 1.0;
    goal.goals[0].pose.orientation.x = 0.0;
    goal.goals[0].pose.orientation.y = 0.0;
    goal.goals[0].pose.orientation.z = 0.0;

    return goal;
  }
};

TEST_F(CostmapPlannerTest, Initialize)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("test.planner_plugin", std::string("GridBased"));
  options.append_parameter_override("test.GridBased.plugin", std::string("nav2_navfn_planner::NavfnPlanner"));

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("costmap_planner_node", options);

  auto planner = std::make_shared<easynav::CostmapPlanner>();

  auto result = planner->initialize(node, "test");
  EXPECT_TRUE(result.has_value());

  auto map = createOccupancyGrid(300, 300, 0.05, 0.0, 0.0);
  auto publisher = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  publisher->on_activate();
  publisher->publish(map); 

  easynav::NavState navstate;

  navstate.odom = createInitialPose();
  navstate.goals = createGoal();

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

  auto path = planner->get_path();
  EXPECT_EQ(path.poses.size(), 0); 

  planner->update(navstate);

  path = planner->get_path();
  EXPECT_NE(path.poses.size(), 0); 
}
