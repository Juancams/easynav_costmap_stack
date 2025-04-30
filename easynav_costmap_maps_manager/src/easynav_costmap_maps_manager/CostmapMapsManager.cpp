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
/// \brief Implementation of the CostmapMapsManager class.

#include <expected>
#include "easynav_costmap_maps_manager/CostmapMapsManager.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "nav2_map_server/map_io.hpp"

namespace easynav
{

std::expected<void, std::string>
CostmapMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  std::string package_name, map_path_file;
  node->declare_parameter(plugin_name + ".package", package_name);
  node->declare_parameter(plugin_name + ".map_path_file", map_path_file);

  node->get_parameter(plugin_name + ".package", package_name);
  node->get_parameter(plugin_name + ".map_path_file", map_path_file);

  nav2_map_server::LoadParameters load_parameters;

  if (package_name != "" && map_path_file != "") {
    std::string pkgpath;
    try {
      pkgpath = ament_index_cpp::get_package_share_directory(package_name);
    } catch(ament_index_cpp::PackageNotFoundError & ex) {
      return std::unexpected("Package " + package_name + " not found. Error: " + ex.what());
    }

    std::string map_path = pkgpath + "/" + map_path_file;

    try {
      load_parameters = nav2_map_server::loadMapYaml(map_path);
    } catch (std::exception & ex) {
      return std::unexpected("Error loading map from " + map_path + ": " + ex.what());
    }
  }

  nav_msgs::msg::OccupancyGrid map;
  
  try {
    nav2_map_server::loadMapFromFile(load_parameters, map);
  } catch (std::exception & ex) {
    return std::unexpected("Error loading map from file: " + std::string(ex.what()));
  }

  static_map_ = std::make_shared<Costmap>(map);
  dynamic_map_ = std::make_shared<Costmap>(map);

  static_costmap_pub_ = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
    node,
    static_map_.get(),
    "map", 
    "/map",     
    true);

  dynamic_costmap_pub_ = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
    node,
    dynamic_map_.get(),
    "map", 
    "/dynamic_map",     
    true);

  static_costmap_pub_->on_activate();
  dynamic_costmap_pub_->on_activate();

  static_costmap_pub_->publishCostmap();

  return {};
}

std::shared_ptr<MapsTypeBase>
CostmapMapsManager::get_static_map()
{
  return static_map_;
}

std::shared_ptr<MapsTypeBase>
CostmapMapsManager::get_dynamyc_map()
{
  return dynamic_map_;
}

void
CostmapMapsManager::update(const NavState & nav_state)
{
  std::memcpy(
    dynamic_map_->getCharMap(),
    static_map_->getCharMap(),
    static_map_->getSizeInCellsX() * static_map_->getSizeInCellsY());

  for (const auto & sensor : nav_state.perceptions) {
    for (const auto & p : sensor->data) {
      unsigned int mx, my;
      if (dynamic_map_->worldToMap(-p.x, -p.y, mx, my)) {
        dynamic_map_->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
  }

  dynamic_costmap_pub_->publishCostmap();
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::CostmapMapsManager, easynav::MapsManagerBase)
