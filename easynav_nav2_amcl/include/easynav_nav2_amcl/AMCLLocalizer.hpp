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
/// \brief Declaration of the AMCLLocalizer class, a default plugin implementation for localization.

#ifndef EASYNAV_NAV2_AMCL__AMCL_LOCALIZER_HPP_
#define EASYNAV_NAV2_AMCL__AMCL_LOCALIZER_HPP_

#include <expected>

#include "nav_msgs/msg/odometry.hpp"
#include "easynav_core/LocalizerMethodBase.hpp"
#include "easynav_nav2_amcl/AMCLProxy.hpp"
#include "easynav_nav2_costmap_maps_manager/Costmap.hpp"

namespace easynav
{

/**
 * @class AMCLLocalizer
 * @brief A default "do-nothing" implementation of the LocalizerMethodBase.
 *
 * This class implements the interface required by the Easy Navigation framework
 * for localization but does not perform any actual computation. It is useful as
 * a placeholder, example, or fallback when no real localization plugin is configured.
 */
class AMCLLocalizer : public easynav::LocalizerMethodBase
{
public:
  /**
   * @brief Default constructor.
   */
  AMCLLocalizer() = default;

  /**
   * @brief Default destructor.
   */
  ~AMCLLocalizer() = default;

  /**
   * @brief Initialize the localization method.
   *
   * This override may be used to set up internal resources. By default, it simply succeeds.
   *
   * @return std::expected<void, std::string> Returns success or an error message.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Retrieve the current robot odometry.
   *
   * Returns the last stored odometry message representing the estimated robot pose.
   * This method should not perform any actual localization computation.
   *
   * @return nav_msgs::msg::Odometry Current localization estimate.
   */
  [[nodiscard]] virtual nav_msgs::msg::Odometry get_odom() override;

  /**
   * @brief Updates the localization estimate based on the current navigation state.
   *
   * This method is intended to run the localization logic and update the odometry.
   * In this dummy implementation, it does nothing.
   *
   * @param nav_state The current navigation state of the system.
   */
  virtual void update_rt(const NavState & nav_state) override;

  /**
   * @brief Updates the localization estimate based on the current navigation state.
   *
   * This method is intended to run the localization logic and update the odometry.
   * In this dummy implementation, it does nothing.
   *
   * @param nav_state The current navigation state of the system.
   */
  virtual void update(const NavState & nav_state) override;

private:
  /**
   * @brief Subscription to the initial pose topic.
   *
   * This subscription listens for messages that provide the initial pose of the robot.
   */
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr
    initial_pose_sub_;
  /**
   * @brief Internal representation of the robot's current odometry.
   *
   * Stores the estimated position and velocity of the robot.
   */
  nav_msgs::msg::Odometry odom_ {};

    /**
     * @brief Pointer to the AMCL proxy object.
     *
     * This proxy is used to interact with the AMCL localization algorithm.
     */
  std::shared_ptr<AMCLProxy> amcl_proxy {nullptr};

  /**
   * @brief Pointer to the costmap object.
   *
   * This costmap is used to represent the environment and obstacles.
   */
  std::shared_ptr<Costmap> map_{nullptr};
};

}  // namespace easynav

#endif  // EASYNAV_NAV2_AMCL__AMCL_LOCALIZER_HPP_
