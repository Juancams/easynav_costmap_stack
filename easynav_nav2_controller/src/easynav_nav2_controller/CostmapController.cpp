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
/// \brief Implementation of the CostmapController class.

#include <expected>

#include "easynav_nav2_controller/CostmapController.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "easynav_nav2_costmap_maps_manager/Costmap.hpp"

namespace easynav
{

std::expected<void, std::string> CostmapController::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  std::string progress_checker_plugin;
  node->declare_parameter(plugin_name + ".progress_checker_plugin", progress_checker_plugin);
  node->get_parameter(plugin_name + ".progress_checker_plugin", progress_checker_plugin);
  if (progress_checker_plugin.empty()) {
    return std::unexpected("No progress checker plugin specified.");
  }

  std::string goal_checker_plugin;
  node->declare_parameter(plugin_name + ".goal_checker_plugin", goal_checker_plugin);
  node->get_parameter(plugin_name + ".goal_checker_plugin", goal_checker_plugin);
  if (goal_checker_plugin.empty()) {
    return std::unexpected("No goal checker plugin specified.");
  }

  std::string controller_plugin;
  node->declare_parameter(plugin_name + ".controller_plugin", controller_plugin);
  node->get_parameter(plugin_name + ".controller_plugin", controller_plugin);
  if (controller_plugin.empty()) {
    return std::unexpected("No controller plugin specified.");
  }

  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
    "local_costmap", node->get_namespace(), node->get_parameter("use_sim_time").as_bool());

  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  costmap_ros_->configure();

  progress_checker_loader_ =
    std::make_unique<pluginlib::ClassLoader<nav2_core::ProgressChecker>>(
    "nav2_core", "nav2_core::ProgressChecker");

  goal_checker_loader_ =
    std::make_unique<pluginlib::ClassLoader<nav2_core::GoalChecker>>(
    "nav2_core", "nav2_core::GoalChecker");

  lp_loader_ =
    std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
    "nav2_core", "nav2_core::Controller");

  std::string progress_checker_plugin_name;
  const std::string progress_checker_plugin_param_name = plugin_name + "." +
    progress_checker_plugin + ".plugin";
  node->declare_parameter(progress_checker_plugin_param_name, progress_checker_plugin_name);
  node->get_parameter(progress_checker_plugin_param_name, progress_checker_plugin_name);
  std::string progress_checker_full_name = plugin_name + "." + progress_checker_plugin;

  try {
    nav2_core::ProgressChecker::Ptr progress_checker =
      progress_checker_loader_->createUniqueInstance(progress_checker_plugin_name);
    progress_checker->initialize(node, progress_checker_full_name);
    progress_checker_ = progress_checker;
  } catch (const pluginlib::LibraryLoadException & ex) {
    return std::unexpected("Failed to load plugin: " + progress_checker_plugin_name + ". Error: " +
        ex.what());
  } catch (const pluginlib::CreateClassException & ex) {
    return std::unexpected("Failed to create instance for plugin: " + progress_checker_plugin_name +
        ". Error: " +
        ex.what());
  }

  std::string goal_checker_plugin_name;
  const std::string goal_checker_plugin_param_name = plugin_name + "." + goal_checker_plugin +
    ".plugin";
  node->declare_parameter(goal_checker_plugin_param_name, goal_checker_plugin_name);
  node->get_parameter(goal_checker_plugin_param_name, goal_checker_plugin_name);
  std::string goal_checker_full_name = plugin_name + "." + goal_checker_plugin;

  try {
    nav2_core::GoalChecker::Ptr goal_checker =
      goal_checker_loader_->createUniqueInstance(goal_checker_plugin_name);
    goal_checker->initialize(node, goal_checker_full_name, costmap_ros_);
    goal_checker_ = goal_checker;
  } catch (const pluginlib::LibraryLoadException & ex) {
    return std::unexpected("Failed to load plugin: " + goal_checker_plugin_name + ". Error: " +
        ex.what());
  } catch (const pluginlib::CreateClassException & ex) {
    return std::unexpected("Failed to create instance for plugin: " + goal_checker_plugin_name +
        ". Error: " +
        ex.what());
  }

  std::string controller_plugin_name;
  const std::string controller_plugin_param_name = plugin_name + "." + controller_plugin +
    ".plugin";
  node->declare_parameter(controller_plugin_param_name, controller_plugin_name);
  node->get_parameter(controller_plugin_param_name, controller_plugin_name);
  std::string controller_full_name = plugin_name + "." + controller_plugin;

  try {
    nav2_core::Controller::Ptr controller =
      lp_loader_->createUniqueInstance(controller_plugin_name);
    controller->configure(node, controller_full_name, costmap_ros_->getTfBuffer(), costmap_ros_);
    controller->activate();
    controller_ = controller;
  } catch (const pluginlib::LibraryLoadException & ex) {
    return std::unexpected("Failed to load plugin: " + controller_plugin_name + ". Error: " +
        ex.what());
  } catch (const pluginlib::CreateClassException & ex) {
    return std::unexpected("Failed to create instance for plugin: " + controller_plugin_name +
        ". Error: " +
        ex.what());
  }

  return {};
}

CostmapController::~CostmapController()
{
  if (costmap_activated_) {
    costmap_ros_->deactivate();
    costmap_ros_->cleanup();
    costmap_activated_ = false;
  }

  if (costmap_thread_) {
    costmap_thread_.reset();
  }

  costmap_ros_.reset();
  controller_.reset();
  progress_checker_loader_.reset();
  progress_checker_.reset();
  goal_checker_loader_.reset();
  goal_checker_.reset();
  lp_loader_.reset();
}

geometry_msgs::msg::TwistStamped CostmapController::get_cmd_vel()
{
  return cmd_vel_;
}

void CostmapController::update_rt(const NavState & nav_state)
{
  if (!costmap_activated_) {
    costmap_ros_->activate();
    costmap_activated_ = true;
  }

  if (nav_state.path.poses.empty()) {
    return;
  }

  geometry_msgs::msg::PoseStamped current;
  current.header.frame_id = nav_state.odom.header.frame_id;
  current.header.stamp = nav_state.odom.header.stamp;
  current.pose = nav_state.odom.pose.pose;

  geometry_msgs::msg::Twist twist;
  twist.linear.x = nav_state.odom.twist.twist.linear.x;
  twist.linear.y = nav_state.odom.twist.twist.linear.y;
  twist.angular.z = nav_state.odom.twist.twist.angular.z;

  controller_->setPlan(nav_state.path);
  cmd_vel_ = controller_->computeVelocityCommands(current, twist,
      goal_checker_.get());
  cmd_vel_.header.stamp = nav_state.odom.header.stamp;
  cmd_vel_.header.frame_id = nav_state.odom.header.frame_id;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::CostmapController, easynav::ControllerMethodBase)
