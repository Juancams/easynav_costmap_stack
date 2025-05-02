# easynav_costmap_stack

## CostmapMapsManager Plugin for Easy Navigation

The `CostmapMapsManager` plugin provides an advanced ROS 2-compatible map management component for the Easy Navigation (EasyNav) framework. It builds upon the `nav2_costmap_2d::Costmap2D` API and supports static and dynamic layers, integrating perception data to maintain an up-to-date costmap used for navigation.

---

### Features

- Loads static maps from disk (`.yaml` + `.pgm`) using Nav2-compatible formats.
- Publishes static and dynamic maps as `nav_msgs/msg/OccupancyGrid`.
- Applies runtime updates to the dynamic costmap via sensor-based perception.
- Accepts map updates via incoming `OccupancyGrid` messages.
- Provides a service to save the current costmap as a PGM+YAML pair.
- Fully compatible with `pluginlib` for dynamic loading into EasyNav.

---

### Parameters

Parameters must be placed under the plugin’s namespace (e.g., `my_plugin.package`, `my_plugin.map_path_file`):

| Name                        | Type   | Description                                                      | Default |
|-----------------------------|--------|------------------------------------------------------------------|---------|
| `plugin_name.package`       | string | Name of the ROS 2 package where the static `.yaml` map resides. | `""`    |
| `plugin_name.map_path_file` | string | Path to the YAML map file (relative to the specified package).   | `""`    |

When both are set, the plugin loads the costmap from the corresponding `.yaml` file (which must reference a `.pgm` file).

---

### Topics

| Topic                                 | Type                              | Direction | Description                                     |
|---------------------------------------|-----------------------------------|-----------|-------------------------------------------------|
| `<node>/<plugin_name>/map`            | `nav_msgs/msg/OccupancyGrid`      | Publisher | Static costmap loaded at initialization.        |
| `<node>/<plugin_name>/dynamic_map`    | `nav_msgs/msg/OccupancyGrid`      | Publisher | Dynamic costmap layer updated via perception.   |
| `<node>/<plugin_name>/incoming_map`   | `nav_msgs/msg/OccupancyGrid`      | Subscriber| Replaces current static map at runtime.         |

---

### Services

| Service                                | Type                     | Description                                          |
|----------------------------------------|--------------------------|------------------------------------------------------|
| `<node>/<plugin_name>/savemap`         | `std_srvs/srv/Trigger`   | Saves the current static costmap to the original file path (PGM). |

---

### Map Format

The plugin uses standard ROS 2 Nav2-compatible map files:

- A `.yaml` file referencing:
  - `image`: path to the `.pgm` file
  - `resolution`, `origin`, `negate`, `occupied_thresh`, `free_thresh`
- A `.pgm` file in `P2` format where:
  - `0` means free
  - `254` means lethal
  - `205` means inflated/obstacle buffer
  - `-1` (255) means unknown

---

### Example Plugin Configuration

```yaml
maps_manager_node:
  ros__parameters:
    maps_manager_plugins:
      - costmap
    costmap:
      package: "my_maps_pkg"
      map_path_file: "maps/warehouse.yaml"
```

---

### Example Usage of Save Service

To save the current static costmap to disk:

```bash
ros2 service call /<node>/costmap/savemap std_srvs/srv/Trigger "{}"
```

---

### License

This plugin is part of the [Easy Navigation (EasyNav)](https://github.com/IntelligentRoboticsLab/easy_navigation) framework and is released under the GNU General Public License v3.0.
