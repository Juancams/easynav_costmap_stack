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
/// \brief Implementation of the AMCL Method class.

#include <expected>
#include "easynav_amcl/AmclMethod.hpp"

namespace easynav
{

std::expected<void, std::string>
AmclMethod::on_initialize()
{
  amcl_proxy = std::make_shared<AmclProxy>(get_node(), get_plugin_name());
  amcl_proxy->initialize();

  initial_pose_sub_ =
    get_node()->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&AmclProxy::on_initial_pose_received, amcl_proxy.get(), std::placeholders::_1));

  return {};
}

nav_msgs::msg::Odometry AmclMethod::get_odom()
{
  return odom_;
}

void
AmclMethod::update_rt(const NavState & nav_state)
{
  (void)nav_state;
}

void
AmclMethod::update(const NavState & nav_state)
{
  if (!map_) {
    try {
      map_ = std::dynamic_pointer_cast<Costmap>(nav_state.maps.at("costmap.static"));
      auto occ_grid_map = map_->to_occupancy_grid();
      amcl_proxy->set_map(std::make_shared<nav_msgs::msg::OccupancyGrid>(occ_grid_map));

    } catch (const std::out_of_range & e) {
      return;
    }
  }

  std::optional<rclcpp::Time> latest_stamp;
  for (const auto & p : nav_state.perceptions) {
    if (p->stamp > latest_stamp) {
      latest_stamp = p->stamp;
    }
  }

  auto fused = PerceptionsOpsView(nav_state.perceptions)
    .downsample(map_->getResolution())
    .fuse("map")
    ->as_points(0);

  const float angle_min = -M_PI;
  const float angle_max = M_PI;
  const float angle_increment = 0.005f;
  const size_t num_ranges = std::ceil((angle_max - angle_min) / angle_increment);
  const float range_min = 0.05f;
  const float range_max = 10.0f;

  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = latest_stamp.value_or(get_node()->now());
  scan.header.frame_id = "map";
  scan.ranges.resize(num_ranges, range_max + 1.0f);
  scan.angle_min = angle_min;
  scan.angle_max = angle_max;
  scan.angle_increment = angle_increment;
  scan.range_min = range_min;
  scan.range_max = range_max;

  for (const auto & p : fused) {
    const float x = p.x;
    const float y = p.y;
    const float r = std::hypot(x, y);

    if (r < range_min || r > range_max) {continue;}

    const float angle = std::atan2(y, x);
    if (angle < angle_min || angle > angle_max) {continue;}

    const size_t index = static_cast<size_t>((angle - angle_min) / angle_increment);
    if (index < num_ranges) {
      scan.ranges[index] = std::min(scan.ranges[index], r);
    }
  }

  amcl_proxy->predict(std::make_shared<sensor_msgs::msg::LaserScan>(scan));
  amcl_proxy->correct(odom_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::AmclMethod, easynav::LocalizerMethodBase)
