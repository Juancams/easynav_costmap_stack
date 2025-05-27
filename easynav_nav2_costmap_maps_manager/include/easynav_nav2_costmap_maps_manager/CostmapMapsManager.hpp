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
/// \brief Declaration of the CostmapMapsManager method.

#ifndef EASYNAV_COSTMAP__COSTMAPMAPMANAGER_HPP_
#define EASYNAV_COSTMAP__COSTMAPMAPMANAGER_HPP_

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <utility>  // std::pair
#include <fstream>
#include <sstream>

#include "easynav_core/MapsManagerBase.hpp"
#include "easynav_common/types/MapTypeBase.hpp"
#include "easynav_nav2_costmap_maps_manager/Costmap.hpp"

#include "nav2_costmap_2d/costmap_2d_publisher.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "pluginlib/class_loader.hpp"

namespace easynav
{

/**
 * @class CostmapMapsManager
 * @brief Implementation of the MapsManagerBase that uses costmaps.
 *
 * This class manages both static and dynamic maps using costmaps.
 * It provides facilities to update the maps from perceptions and export them as OccupancyGrid.
 */
class CostmapMapsManager : public easynav::MapsManagerBase
{
public:
  CostmapMapsManager() = default;
  ~CostmapMapsManager() = default;

  /**
   * @brief Initializes the map manager.
   *
   * Can be overridden to perform custom initialization such as loading parameters
   * or setting up publishers and services.
   *
   * @return std::expected indicating success or failure message.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Returns the current static map.
   *
   * @return Shared pointer to the static SimpleMap instance.
   */
  [[nodiscard]] virtual std::map<std::string, std::shared_ptr<MapsTypeBase>> get_maps() override;

  /**
   * @brief Updates the map based on the current navigation state.
   *
   * This method uses new perceptions from the navigation state to update
   * the internal static or dynamic map representation.
   *
   * @param nav_state The current navigation state, including new perceptions.
   */
  virtual void update(const NavState & nav_state) override;

  /**
   * @brief Replaces the current static map.
   *
   * @param new_map Shared pointer to a new map object. Must be of type Costmap.
   */
  void set_map(std::shared_ptr<MapsTypeBase> new_map);

protected:
  /**
   * @brief Full path to the map file.
   */
  std::string map_path_;

private:
  /**
   * @brief Static costmap representation.
   */
  std::shared_ptr<Costmap> static_map_;

  /**
   * @brief Static map message used for publishing.
   */
  nav_msgs::msg::OccupancyGrid static_map_msgs_;

  /**
   * @brief Publisher to publish the occupancy grid representation of the static map.
   */
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_pub_;

  /**
   * @brief Service to trigger saving the static map to disk.
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr savemap_srv_;

  /**
   * @brief Subscriber for external incoming static map updates.
   */
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr incoming_map_sub_;
};

}  // namespace easynav

#endif  // EASYNAV_COSTMAP__COSTMAPMAPMANAGER_HPP_
