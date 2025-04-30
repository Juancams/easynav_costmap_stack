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

#ifndef EASYNAV__COSTMAP_HPP_
#define EASYNAV__COSTMAP_HPP_

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <utility>
#include <fstream>
#include <sstream>

#include "easynav_common/types/MapTypeBase.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace easynav
{

/**
 * @class Costmap
 * @brief A 2D costmap provides a mapping between points in the world and their associated "costs".
 *
 * This class is a wrapper around the nav2_costmap_2d::Costmap2D class.
 * It provides a simple interface for creating and manipulating costmaps.
 */
class Costmap : public MapsTypeBase, public nav2_costmap_2d::Costmap2D
{
public:
  /**
   * @brief Constructor for a costmap
   * @param map The OccupancyGrid map to create costmap 2D
   */
  inline Costmap(const nav_msgs::msg::OccupancyGrid & map)
  : nav2_costmap_2d::Costmap2D(map) {}
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__COSTMAP_HPP_
