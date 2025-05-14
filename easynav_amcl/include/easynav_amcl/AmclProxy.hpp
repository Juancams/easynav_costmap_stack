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
/// \brief Declaration of the AmclProxy class, a proxy for the AMCL localization algorithm.

#ifndef EASYNAV_AMCL__AMCL_PROXY_HPP_
#define EASYNAV_AMCL__AMCL_PROXY_HPP_

#include "nav2_amcl/amcl_node.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/string_utils.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace easynav
{

using nav2_util::geometry_utils::orientationAroundZAxis;
using namespace std::chrono_literals;

/**
 * @class AmclProxy
 * @brief Proxy class extending nav2_amcl::AmclNode to allow integration with custom systems.
 *
 * This class overrides and extends the behavior of AMCL for use in the Easy Navigation (EasyNav) system.
 * It provides hooks for prediction, correction, and pose/particle publishing with additional data handling.
 */
class AmclProxy : public nav2_amcl::AmclNode
{
public:
  /**
   * @brief Construct a new AmclProxy object.
   *
   * @param parent_node The parent lifecycle node used for initialization.
   */
  AmclProxy(const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent_node, 
    const std::string & plugin_name);

  /**
   * @brief Default destructor.
   */
  ~AmclProxy() override = default;

  /**
   * @brief Initialize parameters and internal state.
   */
  void initialize();

  /**
   * @brief Callback for receiving an initial pose manually.
   *
   * @param msg The initial pose with covariance.
   */
  void on_initial_pose_received(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  /**
   * @brief Set the static occupancy grid map to be used by AMCL.
   *
   * @param map_msg Occupancy grid map message.
   */
  void set_map(
    const nav_msgs::msg::OccupancyGrid::SharedPtr & map_msg);

  /**
   * @brief Perform a prediction step using incoming laser scan data.
   *
   * @param laser_scan Shared pointer to the current laser scan message.
   */
  void predict(const sensor_msgs::msg::LaserScan::SharedPtr laser_scan);

  /**
   * @brief Perform a correction step and estimate the current robot pose.
   *
   * This function updates the particle filter, calculates the best pose hypothesis,
   * computes the `map -> odom` transform, and populates an Odometry message with the result.
   *
   * @param odom_msg The odometry message to be filled with the estimated pose and twist.
   */
  void correct(nav_msgs::msg::Odometry & odom_msg);

  /**
   * @brief Publish the current particle cloud.
   *
   * @param set Pointer to the current particle set.
   */
  void publishParticleCloud(const pf_sample_set_t * set);

  /**
   * @brief Publish the estimated AMCL pose with covariance.
   *
   * @param laser_scan The laser scan used during this update.
   * @param hyps The list of generated pose hypotheses.
   * @param max_weight_hyp Index of the highest-weight hypothesis.
   */
  void publishAmclPose(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp);

private:
  /**
   * @brief Load and declare custom AMCL parameters.
   */
  void init_parameters();

  /**
   * @brief Publisher for the particle cloud (`/particle_cloud`).
   */
  rclcpp::Publisher<nav2_msgs::msg::ParticleCloud>::SharedPtr particle_cloud_pub_;

  /**
   * @brief Publisher for the estimated pose (`/amcl_pose`).
   */
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;

  /**
   * @brief Whether to force pose and transform publication even if unchanged.
   */
  bool force_publication_{false};

  /**
   * @brief Index of the active laser sensor used in updates.
   */
  int laser_index_{0};

  /**
   * @brief Last estimated pose from the particle filter.
   */
  pf_vector_t pose_;

  /**
   * @brief Last received laser scan message.
   */
  sensor_msgs::msg::LaserScan::SharedPtr scan_;

  /**
   * @brief Reference to the parent lifecycle node.
   */
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node_;

  /**
   * @brief Name of the plugin used for AMCL.
   */
  std::string plugin_name_;
};

}  // namespace easynav

#endif  // EASYNAV_AMCL__AMCL_PROXY_HPP_
