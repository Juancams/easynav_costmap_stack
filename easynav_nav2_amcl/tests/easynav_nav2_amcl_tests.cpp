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

#include "easynav_nav2_amcl/AMCLLocalizer.hpp"
#include "easynav_nav2_amcl/AMCLProxy.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


class AMCLLocalizerTest : public ::testing::Test
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

    return createOccupancyGrid(meta_data);
  }

  nav_msgs::msg::OccupancyGrid createOccupancyGrid(const nav_msgs::msg::MapMetaData & meta_data)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.info = meta_data;
    grid.data.resize(meta_data.width * meta_data.height, -1);
    return grid;
  }

};

TEST_F(AMCLLocalizerTest, Initialize)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("amcl_localizer_node");
  auto localizer = std::make_shared<easynav::AMCLLocalizer>();

  auto result = localizer->initialize(node, "test");
  EXPECT_TRUE(result.has_value());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  easynav::NavState navstate;
  localizer->update(navstate);

  auto odom = localizer->get_odom();
  EXPECT_NEAR(odom.pose.pose.position.x, 0.0, 1e-3);
  EXPECT_NEAR(odom.pose.pose.position.y, 0.0, 1e-3);
  EXPECT_NEAR(odom.pose.pose.orientation.w, 1.0, 1e-3);

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub =
    node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::SystemDefaultsQoS());

  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  initial_pose.header.frame_id = "map";
  initial_pose.header.stamp = rclcpp::Time(0);
  initial_pose.pose.pose.position.x = 2.0;
  initial_pose.pose.pose.position.y = 3.0;
  initial_pose.pose.pose.position.z = 0.0;
  initial_pose.pose.pose.orientation.w = 1.0;  // No rotation
  initial_pose.pose.pose.orientation.x = 0.0;
  initial_pose.pose.pose.orientation.y = 0.0;
  initial_pose.pose.pose.orientation.z = 0.0;
  initial_pose.pose.covariance.fill(0.0);

  initial_pose_pub->publish(initial_pose);

  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  nav_msgs::msg::OccupancyGrid map = createOccupancyGrid(30, 30, 0.1, -1.5, -1.5);
  auto static_map = std::make_shared<easynav::Costmap>(map);
  navstate.maps["costmap"] = static_map;

  auto perception = std::make_shared<easynav::Perception>();

  perception->data.points.resize(6);
  perception->data.points[0].x = 1.0;
  perception->data.points[0].y = 1.0;
  perception->data.points[0].z = 0.2;
  perception->data.points[1].x = -1.0;
  perception->data.points[1].y = -1.0;
  perception->data.points[1].z = 0.2;
  perception->data.points[2].x = -10.0;
  perception->data.points[2].y = -1.0;
  perception->data.points[2].z = 0.2;
  perception->data.points[3].x = 10.0;
  perception->data.points[3].y = -1.0;
  perception->data.points[3].z = 0.2;
  perception->data.points[4].x = 1.0;
  perception->data.points[4].y = -10.0;
  perception->data.points[4].z = 0.2;
  perception->data.points[5].x = 1.0;
  perception->data.points[5].y = 10.0;
  perception->data.points[5].z = 0.2;

  perception->stamp = rclcpp::Time(0);
  perception->frame_id = "map";
  perception->valid = true;
  navstate.perceptions.push_back(perception);

  localizer->update(navstate);

  odom = localizer->get_odom();
  EXPECT_NEAR(odom.pose.pose.position.x, 2.0, 1e-3);
  EXPECT_NEAR(odom.pose.pose.position.y, 3.0, 1e-3);
  EXPECT_NEAR(odom.pose.pose.orientation.w, 1.0, 1e-3);
}
