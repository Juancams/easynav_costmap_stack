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
/// \brief Declaration of the CostmapPlanner class

#ifndef EASYNAV_NAV2_PLANNER__COSTMAPPLANNER_HPP_
#define EASYNAV_NAV2_PLANNER__COSTMAPPLANNER_HPP_

#include <expected>

#include "nav_msgs/msg/path.hpp"
#include "easynav_core/PlannerMethodBase.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace easynav
{


class CostmapPlanner : public easynav::PlannerMethodBase
{
public:
  /**
   * @brief Default constructor.
   */
  CostmapPlanner() = default;

  /**
   * @brief Destructor.
   */
  ~CostmapPlanner();

  /**
   * @brief Optional initialization logic.
   *
   * Called after the base initialize method.
   *
   * @return std::expected<void, std::string> Success or error message.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Get the current path.
   *
   * Returns the most recent (or default) path.
   *
   * @return nav_msgs::msg::Path The stored or placeholder path.
   */
  [[nodiscard]] virtual nav_msgs::msg::Path get_path() override;

  /**
   * @brief CostmapPlanner update method.
   *
   * Called to update the internal path based on the current navigation state.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update(const NavState & nav_state) override;

private:
  /**.
   * @brief Stored path message
   */
  nav_msgs::msg::Path path_ {};

  /**
   * @brief Plugin loader for the costmap planner.
   */
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::GlobalPlanner>> costmap_planner_loader_ {};

  /**
   * @brief Pointer to the planner instance.
   */
  nav2_core::GlobalPlanner::Ptr planner_ {};

  /**
   * @brief Pointer to the costmap ros instance.
   */
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_ {};

  /**
   * @brief Thread for the costmap node.
   */
  std::unique_ptr<nav2_util::NodeThread> costmap_thread_;

  /**
   * @brief TF buffer for coordinate transformations.
   */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  /**
   * @brief Publisher for the planned path.
   */
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  /**
   * @brief Flag indicating whether the static map has been activated.
   */
  bool costmap_activated_ {false};
};

}  // namespace easynav

#endif  // EASYNAV_NAV2_PLANNER__COSTMAPPLANNER_HPP_
