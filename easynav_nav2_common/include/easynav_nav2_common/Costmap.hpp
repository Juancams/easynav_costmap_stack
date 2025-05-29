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
/// \brief Declaration of the Costmap type.

#ifndef EASYNAV_COSTMAP__COSTMAP_HPP_
#define EASYNAV_COSTMAP__COSTMAP_HPP_

#include "easynav_common/types/MapTypeBase.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace easynav
{

/**
 * @class Costmap
 * @brief A 2D costmap that represents spatial information as navigation costs.
 *
 * This class extends both nav2_costmap_2d::Costmap2D and MapsTypeBase.
 * It wraps a nav_msgs::msg::OccupancyGrid into a costmap structure for efficient access
 * and processing in navigation tasks.
 */
class Costmap : public MapsTypeBase, public nav2_costmap_2d::Costmap2D
{
public:
  /**
   * @brief Constructor that initializes the costmap from an OccupancyGrid message.
   *
   * This converts a standard ROS 2 OccupancyGrid message into a costmap usable
   * by Nav2-compatible tools and planners.
   *
   * @param map The input OccupancyGrid map.
   */
  inline Costmap(const nav_msgs::msg::OccupancyGrid & map)
  : nav2_costmap_2d::Costmap2D(map) {}


  /**
   * @brief Method to get the costmap as an OccupancyGrid message.
   *
   * @return The OccupancyGrid representation of the costmap.
   */
  nav_msgs::msg::OccupancyGrid to_occupancy_grid() const
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.info.resolution = resolution_;
    grid.info.width = size_x_;
    grid.info.height = size_y_;
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.data.resize(size_x_ * size_y_);
    memcpy(grid.data.data(), costmap_, size_x_ * size_y_ * sizeof(unsigned char));
    return grid;
  }
};

}  // namespace easynav

#endif  // EASYNAV_COSTMAP__COSTMAP_HPP_
