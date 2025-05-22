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

#include "easynav_nav2_costmap_maps_manager/CostmapMapsManager.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "nav2_map_server/map_io.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

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

    map_path_ = pkgpath + "/" + map_path_file;

    try {
      load_parameters = nav2_map_server::loadMapYaml(map_path_);
    } catch (std::exception & ex) {
      return std::unexpected("Error loading map from " + map_path_ + ": " + ex.what());
    }

    nav_msgs::msg::OccupancyGrid map;

    try {
      nav2_map_server::loadMapFromFile(load_parameters, map);
    } catch (std::exception & ex) {
      return std::unexpected("Error loading map from file: " + std::string(ex.what()));
    }

    auto static_map = std::make_shared<Costmap>(map);
    auto dynamic_map = std::make_shared<Costmap>(map);

    set_static_map(static_map);
    set_dynamic_map(dynamic_map);
  }

  incoming_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    node->get_name() + std::string("/") + plugin_name + "/incoming_map",
    rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::UniquePtr msg) {
      auto static_received_map = std::make_shared<Costmap>(*msg);
      auto dynamic_received_map = std::make_shared<Costmap>(*msg);

      set_static_map(static_received_map);
      set_dynamic_map(dynamic_received_map);
    });

  savemap_srv_ = node->create_service<std_srvs::srv::Trigger>(
    node->get_name() + std::string("/") + plugin_name + "/savemap",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void)request;
      if (!static_map_->saveMap(map_path_)) {
        response->success = false;
        response->message = "Failed to save map to: " + map_path_;
      } else {
        response->success = true;
        response->message = "Map successfully saved to: " + map_path_;
      }
    });

  return {};
}

std::map<std::string, std::shared_ptr<MapsTypeBase>>
CostmapMapsManager::get_maps()
{
  std::map<std::string, std::shared_ptr<MapsTypeBase>> ret;
  ret["costmap.static"] = static_map_;
  ret["costmap.dynamic"] = dynamic_map_;

  return ret;
}

void
CostmapMapsManager::set_static_map(std::shared_ptr<MapsTypeBase> new_map)
{
  auto typed_ptr = std::dynamic_pointer_cast<Costmap>(new_map);
  if (!typed_ptr) {
    RCLCPP_WARN(get_node()->get_logger(),
      "CostmapMapsManager::set_static_map: pointer is not a Costmap");
  } else {
    static_map_ = typed_ptr;

    static_costmap_pub_ = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
      get_node(),
      static_map_.get(),
      "map",
      get_node()->get_name() + std::string("/") + get_plugin_name() + "/map",
      true);

    static_costmap_pub_->on_activate();
    static_costmap_pub_->publishCostmap();
  }
}

void
CostmapMapsManager::set_dynamic_map(std::shared_ptr<MapsTypeBase> new_map)
{
  auto typed_ptr = std::dynamic_pointer_cast<Costmap>(new_map);
  if (!typed_ptr) {
    RCLCPP_WARN(get_node()->get_logger(),
      "CostmapMapsManager::set_dynamic_map: pointer is not a Costmap");
  } else {
    dynamic_map_ = typed_ptr;

    dynamic_costmap_pub_ = std::make_shared<nav2_costmap_2d::Costmap2DPublisher>(
      get_node(),
      dynamic_map_.get(),
      "map",
      get_node()->get_name() + std::string("/") + get_plugin_name() + "/dynamic_map",
      true);

    dynamic_costmap_pub_->on_activate();
  }
}

void
CostmapMapsManager::update(const NavState & nav_state)
{
  std::memcpy(
    dynamic_map_->getCharMap(),
    static_map_->getCharMap(),
    static_map_->getSizeInCellsX() * static_map_->getSizeInCellsY());

  auto fused = PerceptionsOpsView(nav_state.perceptions)
    .downsample(static_map_->getResolution())
    .fuse("map")
    ->filter({NAN, NAN, 0.1}, {NAN, NAN, NAN})
    .as_points(0);

  for (const auto & p : fused) {
    unsigned int mx, my;
    if (dynamic_map_->worldToMap(p.x, p.y, mx, my)) {
      dynamic_map_->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  dynamic_costmap_pub_->publishCostmap();
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::CostmapMapsManager, easynav::MapsManagerBase)
