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

/// \file
/// \brief Implementation of the CostmapPlanner class.

#include <expected>

#include "easynav_nav2_planner/CostmapPlanner.hpp"
#include "easynav_nav2_costmap_maps_manager/Costmap.hpp"

namespace easynav
{

std::expected<void, std::string> CostmapPlanner::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "global_costmap", node->get_namespace(), node->get_parameter("use_sim_time").as_bool());

  costmap_ros_->configure();

  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  costmap_planner_loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::GlobalPlanner>>(
    "nav2_core", "nav2_core::GlobalPlanner");

  std::string planner_plugin;
  node->declare_parameter(plugin_name + ".planner_plugin", planner_plugin);
  node->get_parameter(plugin_name + ".planner_plugin", planner_plugin);

  if (planner_plugin.empty()) {
    return std::unexpected("No planner plugin specified.");
  }

  std::string plugin;
  const std::string plugin_param_name = plugin_name + "." + planner_plugin + ".plugin";
  node->declare_parameter(plugin_param_name, plugin);
  node->get_parameter(plugin_param_name, plugin);
  std::string full_name = plugin_name + "." + planner_plugin;

  try {
    planner_ = costmap_planner_loader_->createUniqueInstance(plugin);
    planner_->configure(node, full_name, tf_buffer_, costmap_ros_);
    planner_->activate();
  } catch (const pluginlib::LibraryLoadException & ex) {
    return std::unexpected("Failed to load plugin: " + plugin + ". Error: " + ex.what());
  } catch (const pluginlib::CreateClassException & ex) {
    return std::unexpected("Failed to create instance for plugin: " + plugin + ". Error: " +
        ex.what());
  }

  path_pub_ = node->create_publisher<nav_msgs::msg::Path>("plan", 1);

  return {};
}

nav_msgs::msg::Path CostmapPlanner::get_path()
{
  return path_;
}

void CostmapPlanner::update(const NavState & nav_state)
{
  if (planner_ == nullptr || nav_state.odom.header.frame_id == "") {
    return;
  }

  if (!costmap_activated_) {
    costmap_ros_->activate();
    costmap_activated_ = true;
  }

  if (!nav_state.goals.goals.empty()) {
    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = nav_state.odom.header.frame_id;
    start.header.stamp = nav_state.odom.header.stamp;
    start.pose = nav_state.odom.pose.pose;

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = nav_state.goals.goals[0].header.frame_id;
    goal.header.stamp = nav_state.goals.goals[0].header.stamp;
    goal.pose = nav_state.goals.goals[0].pose;

    path_ = planner_->createPlan(start, goal, []() {return false;});

    if (path_.poses.empty()) {
      RCLCPP_WARN(get_node()->get_logger(), "Path is empty");
      return;
    }

    path_pub_->publish(path_);
  }
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::CostmapPlanner, easynav::PlannerMethodBase)
