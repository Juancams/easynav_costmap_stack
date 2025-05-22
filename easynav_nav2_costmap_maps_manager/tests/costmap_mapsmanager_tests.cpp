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

#include "easynav_nav2_costmap_maps_manager/CostmapMapsManager.hpp"
#include "easynav_nav2_costmap_maps_manager/Costmap.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "tf2_ros/transform_listener.h"

using easynav::CostmapMapsManager;
using easynav::Costmap;
using easynav::NavState;
using easynav::Perception;

class CostmapMapsManagerTest : public ::testing::Test
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

TEST_F(CostmapMapsManagerTest, SetAndGetMap)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("costmap_node2");
  auto manager = std::make_shared<CostmapMapsManager>();
  manager->initialize(node, "test");

  nav_msgs::msg::OccupancyGrid map = createOccupancyGrid(10, 10, 0.1, 0.0, 0.0);
  auto static_map = std::make_shared<Costmap>(map);
  manager->set_static_map(static_map);
  manager->set_dynamic_map(static_map);

  auto st_map = std::dynamic_pointer_cast<Costmap>(manager->get_maps().at("costmap.static"));
  ASSERT_TRUE(st_map != nullptr);
  EXPECT_NEAR(st_map->getResolution(), 0.1, 1e-3);

  auto dyn_map = std::dynamic_pointer_cast<Costmap>(manager->get_maps().at("costmap.dynamic"));
  ASSERT_TRUE(dyn_map != nullptr);
  EXPECT_NEAR(dyn_map->getResolution(), 0.1, 1e-3);
}

TEST_F(CostmapMapsManagerTest, DynamicUpdateModifiesDynamicMap)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("costmap_node3");
  auto tf_buffer = easynav::RTTFBuffer::getInstance(node->get_clock());
  tf2_ros::TransformListener tf_listener(*tf_buffer, node, true);
  auto manager = std::make_shared<CostmapMapsManager>();
  manager->initialize(node, "test");

  nav_msgs::msg::OccupancyGrid map = createOccupancyGrid(30, 30, 0.1, -1.5, -1.5);
  auto static_map = std::make_shared<Costmap>(map);
  manager->set_static_map(static_map);
  manager->set_dynamic_map(static_map);

  easynav::NavState navstate;
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

  manager->update(navstate);

  auto dyn_map = std::dynamic_pointer_cast<Costmap>(manager->get_maps().at("costmap.dynamic"));
  ASSERT_TRUE(dyn_map != nullptr);

  unsigned int mx, my;
  bool found = dyn_map->worldToMap(1.0, 1.0, mx, my);
  ASSERT_TRUE(found);
  EXPECT_EQ(dyn_map->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);

  found = dyn_map->worldToMap(-1.0, -1.0, mx, my);
  ASSERT_TRUE(found);
  EXPECT_EQ(dyn_map->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);

  EXPECT_FALSE(dyn_map->worldToMap(-10.0, -1.0, mx, my));
  EXPECT_FALSE(dyn_map->worldToMap(10.0, -1.0, mx, my));
  EXPECT_FALSE(dyn_map->worldToMap(1.0, -10.0, mx, my));
  EXPECT_FALSE(dyn_map->worldToMap(1.0, 10.0, mx, my));
}

TEST_F(CostmapMapsManagerTest, IncomingOccupancyGridUpdatesMaps)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("costmap_sub_node");
  auto manager = std::make_shared<CostmapMapsManager>();
  manager->initialize(node, "subtest");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub =
    node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "costmap_sub_node/subtest/incoming_map", rclcpp::QoS(1).transient_local().reliable());

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = 10;
  grid.info.height = 10;
  grid.info.resolution = 0.2;
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -0.6;
  grid.data.assign(100, 0);
  grid.data[55] = 100;  // Cell (5,5)

  pub->publish(grid);

  executor.spin_some();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  auto static_map = std::dynamic_pointer_cast<Costmap>(manager->get_maps().at("costmap.static"));
  ASSERT_TRUE(static_map != nullptr);

  EXPECT_EQ(static_map->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(static_map->getCost(0, 0), nav2_costmap_2d::FREE_SPACE);
}

class FriendCostmapMapsManager : public CostmapMapsManager {
public:
  void force_path(const std::string & path) {map_path_ = path;}
};

TEST_F(CostmapMapsManagerTest, SavemapServiceWorks)
  {
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("costmap_savemap_node");
    auto manager = std::make_shared<FriendCostmapMapsManager>();
    manager->initialize(node, "save");

    auto map = std::make_shared<Costmap>(createOccupancyGrid(4, 4, 0.5, -1.0, -1.0));
    for (unsigned int i = 0; i < 4; ++i) {
    for (unsigned int j = 0; j < 4; ++j) {
      map->setCost(i, j, nav2_costmap_2d::FREE_SPACE);
    }
    }

    map->setCost(1, 1, nav2_costmap_2d::LETHAL_OBSTACLE);
    map->setCost(2, 2, nav2_costmap_2d::LETHAL_OBSTACLE);

    manager->set_static_map(map);

    const std::string test_map_file = "/tmp/costmap_saved.pgm";
    const std::string service_name = "/costmap_savemap_node/save/savemap";
    manager->force_path(test_map_file);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());

    auto client = node->create_client<std_srvs::srv::Trigger>(service_name);
    ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(request);
    executor.spin_until_future_complete(future);

    auto response = future.get();
    EXPECT_TRUE(response->success);
    EXPECT_NE(response->message.find("saved"), std::string::npos);

    std::ifstream infile(test_map_file);
    ASSERT_TRUE(infile.is_open());

    std::string magic;
    int width = 0, height = 0, max_val = 0;

    infile >> magic >> width >> height >> max_val;
    EXPECT_EQ(magic, "P2");
    EXPECT_EQ(width, 4);
    EXPECT_EQ(height, 4);
    EXPECT_EQ(max_val, 255);

    std::vector<int> map_data;
    int val;
    while (infile >> val) {
    map_data.push_back(val);
    }
    infile.close();

    EXPECT_EQ(map_data.size(), 4 * 4);
    EXPECT_EQ(map_data[5], 254);    // (1,1)
    EXPECT_EQ(map_data[10], 254);   // (2,2)

    infile.close();
    std::remove(test_map_file.c_str());
}
