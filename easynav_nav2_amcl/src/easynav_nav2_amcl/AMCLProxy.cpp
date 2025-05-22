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
/// \brief Implementation of the AMCL Proxy class.

#include "easynav_nav2_amcl/AMCLProxy.hpp"

namespace easynav
{

AMCLProxy::AMCLProxy(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent_node, const
  std::string & plugin_name)
: nav2_amcl::AmclNode(rclcpp::NodeOptions{}), parent_node_(parent_node), plugin_name_(plugin_name)
{
  parent_node_->declare_parameter(plugin_name_ + ".alpha1", rclcpp::ParameterValue(0.2));
  parent_node_->declare_parameter(plugin_name_ + ".alpha2", rclcpp::ParameterValue(0.2));
  parent_node_->declare_parameter(plugin_name_ + ".alpha3", rclcpp::ParameterValue(0.2));
  parent_node_->declare_parameter(plugin_name_ + ".alpha4", rclcpp::ParameterValue(0.2));
  parent_node_->declare_parameter(plugin_name_ + ".alpha5", rclcpp::ParameterValue(0.2));
  parent_node_->declare_parameter(plugin_name_ + ".base_frame_id",
      rclcpp::ParameterValue(std::string("base_footprint")));

  parent_node_->declare_parameter(plugin_name_ + ".beam_skip_distance",
      rclcpp::ParameterValue(0.5));
  parent_node_->declare_parameter(plugin_name_ + ".beam_skip_error_threshold",
      rclcpp::ParameterValue(0.9));
  parent_node_->declare_parameter(plugin_name_ + ".beam_skip_threshold",
      rclcpp::ParameterValue(0.3));
  parent_node_->declare_parameter(plugin_name_ + ".do_beamskip", rclcpp::ParameterValue(false));
  parent_node_->declare_parameter(plugin_name_ + ".global_frame_id",
      rclcpp::ParameterValue(std::string("map")));
  parent_node_->declare_parameter(plugin_name_ + ".lambda_short", rclcpp::ParameterValue(0.1));
  parent_node_->declare_parameter(plugin_name_ + ".laser_likelihood_max_dist",
      rclcpp::ParameterValue(2.0));
  parent_node_->declare_parameter(plugin_name_ + ".laser_max_range", rclcpp::ParameterValue(100.0));
  parent_node_->declare_parameter(plugin_name_ + ".laser_min_range", rclcpp::ParameterValue(-1.0));
  parent_node_->declare_parameter(plugin_name_ + ".laser_model_type",
      rclcpp::ParameterValue(std::string("likelihood_field")));

  parent_node_->declare_parameter(plugin_name_ + ".set_initial_pose",
      rclcpp::ParameterValue(false));
  parent_node_->declare_parameter(plugin_name_ + ".initial_pose.x", rclcpp::ParameterValue(0.0));
  parent_node_->declare_parameter(plugin_name_ + ".initial_pose.y", rclcpp::ParameterValue(0.0));
  parent_node_->declare_parameter(plugin_name_ + ".initial_pose.z", rclcpp::ParameterValue(0.0));
  parent_node_->declare_parameter(plugin_name_ + ".initial_pose.yaw", rclcpp::ParameterValue(0.0));
  parent_node_->declare_parameter(plugin_name_ + ".max_beams", rclcpp::ParameterValue(60));
  parent_node_->declare_parameter(plugin_name_ + ".max_particles", rclcpp::ParameterValue(2000));
  parent_node_->declare_parameter(plugin_name_ + ".min_particles", rclcpp::ParameterValue(500));
  parent_node_->declare_parameter(plugin_name_ + ".odom_frame_id",
      rclcpp::ParameterValue(std::string("odom")));
  parent_node_->declare_parameter(plugin_name_ + ".pf_err", rclcpp::ParameterValue(0.05));
  parent_node_->declare_parameter(plugin_name_ + ".pf_z", rclcpp::ParameterValue(0.99));
  parent_node_->declare_parameter(plugin_name_ + ".recovery_alpha_fast",
      rclcpp::ParameterValue(0.0));
  parent_node_->declare_parameter(plugin_name_ + ".recovery_alpha_slow",
      rclcpp::ParameterValue(0.0));
  parent_node_->declare_parameter(plugin_name_ + ".resample_interval", rclcpp::ParameterValue(1));
  parent_node_->declare_parameter(plugin_name_ + ".robot_model_type",
      rclcpp::ParameterValue("nav2_amcl::DifferentialMotionModel"));

  parent_node_->declare_parameter(plugin_name_ + ".save_pose_rate", rclcpp::ParameterValue(0.5));
  parent_node_->declare_parameter(plugin_name_ + ".sigma_hit", rclcpp::ParameterValue(0.2));
  parent_node_->declare_parameter(plugin_name_ + ".tf_broadcast", rclcpp::ParameterValue(true));
  parent_node_->declare_parameter(plugin_name_ + ".transform_tolerance",
      rclcpp::ParameterValue(1.0));
  parent_node_->declare_parameter(plugin_name_ + ".update_min_a", rclcpp::ParameterValue(0.2));
  parent_node_->declare_parameter(plugin_name_ + ".update_min_d", rclcpp::ParameterValue(0.25));
  parent_node_->declare_parameter(plugin_name_ + ".z_hit", rclcpp::ParameterValue(0.5));
  parent_node_->declare_parameter(plugin_name_ + ".z_max", rclcpp::ParameterValue(0.05));
  parent_node_->declare_parameter(plugin_name_ + ".z_rand", rclcpp::ParameterValue(0.5));
  parent_node_->declare_parameter(plugin_name_ + ".z_short", rclcpp::ParameterValue(0.05));
  parent_node_->declare_parameter(plugin_name_ + ".always_reset_initial_pose",
      rclcpp::ParameterValue(false));
  parent_node_->declare_parameter(plugin_name_ + ".first_map_only", rclcpp::ParameterValue(false));
  parent_node_->declare_parameter(plugin_name_ + ".freespace_downsampling",
      rclcpp::ParameterValue(false));
}

void
AMCLProxy::init_parameters()
{
  double save_pose_rate;
  double tmp_tol;

  parent_node_->get_parameter(plugin_name_ + ".alpha1", alpha1_);
  parent_node_->get_parameter(plugin_name_ + ".alpha2", alpha2_);
  parent_node_->get_parameter(plugin_name_ + ".alpha3", alpha3_);
  parent_node_->get_parameter(plugin_name_ + ".alpha4", alpha4_);
  parent_node_->get_parameter(plugin_name_ + ".alpha5", alpha5_);
  parent_node_->get_parameter(plugin_name_ + ".base_frame_id", base_frame_id_);
  parent_node_->get_parameter(plugin_name_ + ".beam_skip_distance", beam_skip_distance_);
  parent_node_->get_parameter(plugin_name_ + ".beam_skip_error_threshold",
      beam_skip_error_threshold_);
  parent_node_->get_parameter(plugin_name_ + ".beam_skip_threshold", beam_skip_threshold_);
  parent_node_->get_parameter(plugin_name_ + ".do_beamskip", do_beamskip_);
  parent_node_->get_parameter(plugin_name_ + ".global_frame_id", global_frame_id_);
  parent_node_->get_parameter(plugin_name_ + ".lambda_short", lambda_short_);
  parent_node_->get_parameter(plugin_name_ + ".laser_likelihood_max_dist",
      laser_likelihood_max_dist_);
  parent_node_->get_parameter(plugin_name_ + ".laser_max_range", laser_max_range_);
  parent_node_->get_parameter(plugin_name_ + ".laser_min_range", laser_min_range_);
  parent_node_->get_parameter(plugin_name_ + ".laser_model_type", sensor_model_type_);
  parent_node_->get_parameter(plugin_name_ + ".set_initial_pose", set_initial_pose_);
  parent_node_->get_parameter(plugin_name_ + ".initial_pose.x", initial_pose_x_);
  parent_node_->get_parameter(plugin_name_ + ".initial_pose.y", initial_pose_y_);
  parent_node_->get_parameter(plugin_name_ + ".initial_pose.z", initial_pose_z_);
  parent_node_->get_parameter(plugin_name_ + ".initial_pose.yaw", initial_pose_yaw_);
  parent_node_->get_parameter(plugin_name_ + ".max_beams", max_beams_);
  parent_node_->get_parameter(plugin_name_ + ".max_particles", max_particles_);
  parent_node_->get_parameter(plugin_name_ + ".min_particles", min_particles_);
  parent_node_->get_parameter(plugin_name_ + ".odom_frame_id", odom_frame_id_);
  parent_node_->get_parameter(plugin_name_ + ".pf_err", pf_err_);
  parent_node_->get_parameter(plugin_name_ + ".pf_z", pf_z_);
  parent_node_->get_parameter(plugin_name_ + ".recovery_alpha_fast", alpha_fast_);
  parent_node_->get_parameter(plugin_name_ + ".recovery_alpha_slow", alpha_slow_);
  parent_node_->get_parameter(plugin_name_ + ".resample_interval", resample_interval_);
  parent_node_->get_parameter(plugin_name_ + ".robot_model_type", robot_model_type_);
  parent_node_->get_parameter(plugin_name_ + ".save_pose_rate", save_pose_rate);
  parent_node_->get_parameter(plugin_name_ + ".sigma_hit", sigma_hit_);
  parent_node_->get_parameter(plugin_name_ + ".tf_broadcast", tf_broadcast_);
  parent_node_->get_parameter(plugin_name_ + ".transform_tolerance", tmp_tol);
  parent_node_->get_parameter(plugin_name_ + ".update_min_a", a_thresh_);
  parent_node_->get_parameter(plugin_name_ + ".update_min_d", d_thresh_);
  parent_node_->get_parameter(plugin_name_ + ".z_hit", z_hit_);
  parent_node_->get_parameter(plugin_name_ + ".z_max", z_max_);
  parent_node_->get_parameter(plugin_name_ + ".z_rand", z_rand_);
  parent_node_->get_parameter(plugin_name_ + ".z_short", z_short_);
  parent_node_->get_parameter(plugin_name_ + ".first_map_only", first_map_only_);
  parent_node_->get_parameter(plugin_name_ + ".always_reset_initial_pose",
      always_reset_initial_pose_);
  parent_node_->get_parameter(plugin_name_ + ".freespace_downsampling", freespace_downsampling_);

  save_pose_period_ = tf2::durationFromSec(1.0 / save_pose_rate);
  transform_tolerance_ = tf2::durationFromSec(tmp_tol);

  odom_frame_id_ = nav2_util::strip_leading_slash(odom_frame_id_);
  base_frame_id_ = nav2_util::strip_leading_slash(base_frame_id_);
  global_frame_id_ = nav2_util::strip_leading_slash(global_frame_id_);

  last_time_printed_msg_ = now();

  // Semantic checks
  if (laser_likelihood_max_dist_ < 0) {
    RCLCPP_WARN(
      get_logger(), "You've set laser_likelihood_max_dist to be negative,"
      " this isn't allowed so it will be set to default value 2.0.");
    laser_likelihood_max_dist_ = 2.0;
  }
  if (max_particles_ < 0) {
    RCLCPP_WARN(
      get_logger(), "You've set max_particles to be negative,"
      " this isn't allowed so it will be set to default value 2000.");
    max_particles_ = 2000;
  }

  if (min_particles_ < 0) {
    RCLCPP_WARN(
      get_logger(), "You've set min_particles to be negative,"
      " this isn't allowed so it will be set to default value 500.");
    min_particles_ = 500;
  }

  if (min_particles_ > max_particles_) {
    RCLCPP_WARN(
      get_logger(), "You've set min_particles to be greater than max particles,"
      " this isn't allowed so max_particles will be set to min_particles.");
    max_particles_ = min_particles_;
  }

  if (resample_interval_ <= 0) {
    RCLCPP_WARN(
      get_logger(), "You've set resample_interval to be zero or negative,"
      " this isn't allowed so it will be set to default value to 1.");
    resample_interval_ = 1;
  }

  if (always_reset_initial_pose_) {
    initial_pose_is_known_ = false;
  }
}

void AMCLProxy::initialize()
{
  init_parameters();
  initTransforms();
  initParticleFilter();
  initLaserScan();
  initOdometry();

  particle_cloud_pub_ = parent_node_->create_publisher<nav2_msgs::msg::ParticleCloud>(
    "particle_cloud",
    rclcpp::SensorDataQoS());

  pose_pub_ = parent_node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "amcl_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  first_pose_sent_ = false;
  active_ = true;
}

void AMCLProxy::on_initial_pose_received(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  if (active_) {
    handleInitialPose(*msg);
    RCLCPP_INFO(
      get_logger(), "Initial pose received: %6.3f %6.3f",
      msg->pose.pose.position.x,
      msg->pose.pose.position.y);
  } else {
    init_pose_received_on_inactive = true;
    last_published_pose_ = *msg;
  }
}

void AMCLProxy::set_map(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & map_msg)
{
  handleMapMessage(*map_msg);
  first_map_received_ = true;
}

void AMCLProxy::predict(
  const sensor_msgs::msg::LaserScan::SharedPtr laser_scan)
{
  std::lock_guard<std::recursive_mutex> cfl(mutex_);
  scan_ = laser_scan;

  // Since the sensor data is continually being published by the simulator or robot,
  // we don't want our callbacks to fire until we're in the active state
  if (!active_) {return;}
  if (!first_map_received_) {
    if (checkElapsedTime(2s, last_time_printed_msg_)) {
      RCLCPP_WARN(get_logger(), "Waiting for map....");
      last_time_printed_msg_ = now();
    }
    return;
  }

  std::string laser_scan_frame_id = nav2_util::strip_leading_slash(scan_->header.frame_id);
  last_laser_received_ts_ = now();
  laser_index_ = -1;
  geometry_msgs::msg::PoseStamped laser_pose;

  // Do we have the base->base_laser Tx yet?
  // Una vez y fuera del predict
  if (frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end()) {
    if (!addNewScanner(laser_index_, scan_, laser_scan_frame_id, laser_pose)) {
      return;  // could not find transform
    }
  } else {
    // we have the laser pose, retrieve laser index
    laser_index_ = frame_to_laser_[scan_->header.frame_id];
  }

  // Where was the robot when this scan was taken?
  if (!getOdomPose(
      latest_odom_pose_, pose_.v[0], pose_.v[1], pose_.v[2],
      scan_->header.stamp, base_frame_id_))
  {
    RCLCPP_ERROR(get_logger(), "Couldn't determine robot's pose associated with laser scan");
    return;
  }

  pf_vector_t delta = pf_vector_zero();
  force_publication_ = false;
  if (!pf_init_) {
    // Pose at last filter update
    pf_odom_pose_ = pose_;
    pf_init_ = true;

    for (unsigned int i = 0; i < lasers_update_.size(); i++) {
      lasers_update_[i] = true;
    }

    force_publication_ = true;
    resample_count_ = 0;
  } else {
    // Set the laser update flags
    if (shouldUpdateFilter(pose_, delta)) {
      for (unsigned int i = 0; i < lasers_update_.size(); i++) {
        lasers_update_[i] = true;
      }
    }
    if (lasers_update_[laser_index_]) {
      motion_model_->odometryUpdate(pf_, pose_, delta);
    }
    force_update_ = false;
  }
}

void AMCLProxy::correct(nav_msgs::msg::Odometry & odom_msg)
{
  bool resampled = false;

  // If the robot has moved, update the filter
  if (static_cast<std::size_t>(laser_index_) < lasers_update_.size()) {

    if (lasers_update_[laser_index_]) {
      updateFilter(laser_index_, scan_, pose_);

      // Resample the particles
      if (!(++resample_count_ % resample_interval_)) {
        pf_update_resample(pf_, reinterpret_cast<void *>(map_));
        resampled = true;
      }

      pf_sample_set_t * set = pf_->sets + pf_->current_set;
      RCLCPP_INFO(get_logger(), "Num samples: %d\n", set->sample_count);

      if (!force_update_) {
        publishParticleCloud(set);
      }
    }
  }

  if (resampled || force_publication_ || !first_pose_sent_) {
    amcl_hyp_t max_weight_hyps;
    std::vector<amcl_hyp_t> hyps;
    int max_weight_hyp = -1;
    if (getMaxWeightHyp(hyps, max_weight_hyps, max_weight_hyp)) {
      if (scan_ == nullptr) {
        RCLCPP_INFO(get_logger(), "No scan!");
        return;
      }
      publishAmclPose(scan_, hyps, max_weight_hyp);
      calculateMaptoOdomTransform(scan_, hyps, max_weight_hyp);

      odom_msg.header.stamp = scan_->header.stamp;
      odom_msg.header.frame_id = "map";
      odom_msg.child_frame_id = "odom";

      odom_msg.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      odom_msg.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      odom_msg.pose.pose.position.z = 0.0;

      odom_msg.pose.pose.orientation = orientationAroundZAxis(
        hyps[max_weight_hyp].pf_pose_mean.v[2]);

      pf_sample_set_t * set = pf_->sets + pf_->current_set;
      for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
          odom_msg.pose.covariance[6 * i + j] = set->cov.m[i][j];
        }
      }
      odom_msg.pose.covariance[6 * 5 + 5] = set->cov.m[2][2];

      if (tf_broadcast_ == true) {
        // We want to send a transform that is good up until a
        // tolerance time so that odom can be used
        auto stamp = tf2_ros::fromMsg(scan_->header.stamp);
        tf2::TimePoint transform_expiration = stamp + transform_tolerance_;
        sendMapToOdomTransform(transform_expiration);
        sent_first_transform_ = true;
      }
    } else {
      RCLCPP_ERROR(get_logger(), "No pose!");
    }
  } else if (latest_tf_valid_) {
    if (tf_broadcast_ == true) {
      // Nothing changed, so we'll just republish the last transform, to keep
      // everybody happy.
      tf2::TimePoint transform_expiration = tf2_ros::fromMsg(scan_->header.stamp) +
        transform_tolerance_;
      sendMapToOdomTransform(transform_expiration);
    }
  }
}

void
AMCLProxy::publishParticleCloud(const pf_sample_set_t * set)
{
  // If initial pose is not known, AMCL does not know the current pose
  if (!initial_pose_is_known_) {return;}
  auto cloud_with_weights_msg = std::make_unique<nav2_msgs::msg::ParticleCloud>();
  cloud_with_weights_msg->header.stamp = this->now();
  cloud_with_weights_msg->header.frame_id = global_frame_id_;
  cloud_with_weights_msg->particles.resize(set->sample_count);

  for (int i = 0; i < set->sample_count; i++) {
    cloud_with_weights_msg->particles[i].pose.position.x = set->samples[i].pose.v[0];
    cloud_with_weights_msg->particles[i].pose.position.y = set->samples[i].pose.v[1];
    cloud_with_weights_msg->particles[i].pose.position.z = 0;
    cloud_with_weights_msg->particles[i].pose.orientation = orientationAroundZAxis(
      set->samples[i].pose.v[2]);
    cloud_with_weights_msg->particles[i].weight = set->samples[i].weight;
  }

  particle_cloud_pub_->publish(std::move(cloud_with_weights_msg));
}

void
AMCLProxy::publishAmclPose(
  const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
  const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp)
{
  // If initial pose is not known, AMCL does not know the current pose
  if (!initial_pose_is_known_) {
    if (checkElapsedTime(2s, last_time_printed_msg_)) {
      RCLCPP_WARN(
        get_logger(), "AMCL cannot publish a pose or update the transform. "
        "Please set the initial pose...");
      last_time_printed_msg_ = now();
    }
    return;
  }

  auto p = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
  // Fill in the header
  p->header.frame_id = global_frame_id_;
  p->header.stamp = laser_scan->header.stamp;
  // Copy in the pose
  p->pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
  p->pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
  p->pose.pose.orientation = orientationAroundZAxis(hyps[max_weight_hyp].pf_pose_mean.v[2]);
  // Copy in the covariance, converting from 3-D to 6-D
  pf_sample_set_t * set = pf_->sets + pf_->current_set;
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      // p->covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
      p->pose.covariance[6 * i + j] = set->cov.m[i][j];
    }
  }
  p->pose.covariance[6 * 5 + 5] = set->cov.m[2][2];
  float temp = 0.0;
  for (auto covariance_value : p->pose.covariance) {
    temp += covariance_value;
  }
  temp += p->pose.pose.position.x + p->pose.pose.position.y;
  if (!std::isnan(temp)) {
    RCLCPP_DEBUG(get_logger(), "Publishing pose");
    last_published_pose_ = *p;
    first_pose_sent_ = true;
    pose_pub_->publish(std::move(p));
  } else {
    RCLCPP_WARN(
      get_logger(), "AMCL covariance or pose is NaN, likely due to an invalid "
      "configuration or faulty sensor measurements! Pose is not available!");
  }

  RCLCPP_DEBUG(
    get_logger(), "New pose: %6.3f %6.3f %6.3f",
    hyps[max_weight_hyp].pf_pose_mean.v[0],
    hyps[max_weight_hyp].pf_pose_mean.v[1],
    hyps[max_weight_hyp].pf_pose_mean.v[2]);
}

}  // namespace easynav
