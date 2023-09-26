// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// std
#include <list>
#include <memory>
#include <string>
#include <utility>

// romea
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_robot_to_human_localisation_core/robot_to_human_localisation_filter.hpp"


namespace romea
{


//-----------------------------------------------------------------------------
R2HLocalisationFilter::R2HLocalisationFilter(std::shared_ptr<rclcpp::Node> node)
: filter_(nullptr),
  results_(nullptr),
  updater_interfaces_()
{
  make_filter_(node);
  add_proprioceptive_updater_interface_<UpdaterInterfaceTwist>(
    node, "twist_updater", "twist", 10);
  add_proprioceptive_updater_interface_<UpdaterInterfaceLinearSpeed>(
    node, "linear_speed_updater", "twist", 0);
  add_proprioceptive_updater_interface_<UpdaterInterfaceLinearSpeeds>(
    node, "linear_speeds_updater", "twist", 0);
  add_proprioceptive_updater_interface_<UpdaterInterfaceAngularSpeed>(
    node, "angular_speed_updater", "angular_speed", 0);
  add_position_updater_interface_(
    node, "position_updater", "leader_position", 1, "once");
  add_range_updater_interface_(
    node, "range_updater", "range", 10, "always");
  make_results_(node);
}

//-----------------------------------------------------------------------------
LocalisationFSMState R2HLocalisationFilter::get_fsm_state()
{
  return filter_->getFSMState();
}


//-----------------------------------------------------------------------------
void R2HLocalisationFilter::reset()
{
  filter_->reset();
}

//-----------------------------------------------------------------------------
void R2HLocalisationFilter::make_filter_(std::shared_ptr<rclcpp::Node> node)
{
  declare_predictor_parameters(node, 1.0, 1.0, std::numeric_limits<double>::max());
  declare_filter_parameters<FilterType::KALMAN>(node);
  declare_parameter<double>(node, "predictor", "leader_motion_noise_std");

  filter_ = make_filter<Filter, FilterType::KALMAN>(node);

  auto predictor = std::make_unique<Predictor>(
    durationFromSecond(get_predictor_maximal_dead_reckoning_elapsed_time(node)),
    get_predictor_maximal_dead_reckoning_travelled_distance(node),
    get_predictor_maximal_circular_error_probable(node),
    get_parameter<double>(node, "predictor", "leader_motion_noise_std"));

  filter_->registerPredictor(std::move(predictor));
  RCLCPP_INFO_STREAM(node->get_logger(), "filter: started ");
}

//-----------------------------------------------------------------------------
void R2HLocalisationFilter::make_results_(std::shared_ptr<rclcpp::Node> node)
{
  results_ = make_results<Results, FilterType::KALMAN>(node);
}

//-----------------------------------------------------------------------------
template<typename Interface>
void R2HLocalisationFilter::add_proprioceptive_updater_interface_(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name,
  const std::string & topic_name,
  const unsigned int & default_minimal_rate)
{
  declare_proprioceptive_updater_parameters(node, updater_name, default_minimal_rate);

  if (get_updater_minimal_rate(node, updater_name) != 0) {
    using Updater = typename Interface::Updater;
    auto updater = make_proprioceptive_updater<Updater>(
      node,
      updater_name);

    auto plugin = make_updater_interface<Interface>(
      node,
      topic_name,
      filter_,
      std::move(updater));

    updater_interfaces_.push_back(std::move(plugin));
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": started ");
  } else {
    RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": unconfigured ");
  }
}

//-----------------------------------------------------------------------------
void R2HLocalisationFilter::add_range_updater_interface_(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name,
  const std::string & topic_name,
  const unsigned int & default_minimal_rate,
  const std::string & default_trigger_mode)
{
  declare_exteroceptive_updater_parameters(
    node, updater_name, default_minimal_rate, default_trigger_mode);

  declare_parameter<bool>(node, updater_name, "use_constraints");

  auto updater = std::make_unique<UpdaterRange>(
    updater_name,
    get_updater_minimal_rate(node, updater_name),
    toTriggerMode(get_updater_trigger_mode(node, updater_name)),
    get_updater_mahalanobis_distance_rejection_threshold(node, updater_name),
    get_log_filename(node, updater_name),
    get_parameter<bool>(node, updater_name, "use_constraints"));

  auto plugin = make_updater_interface<UpdaterInterfaceRange>(
    node,
    topic_name,
    filter_,
    std::move(updater));

  updater_interfaces_.push_back(std::move(plugin));
  RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": started ");
}


//-----------------------------------------------------------------------------
void R2HLocalisationFilter::add_position_updater_interface_(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name,
  const std::string & topic_name,
  const unsigned int & default_minimal_rate,
  const std::string & default_trigger_mode)
{
  declare_exteroceptive_updater_parameters(
    node, updater_name, default_minimal_rate, default_trigger_mode);

  auto updater = make_exteroceptive_updater<UpdaterPosition, FilterType::KALMAN>(
    node,
    updater_name);

  auto plugin = make_updater_interface<UpdaterInterfacePosition>(
    node,
    topic_name,
    filter_,
    std::move(updater));

  updater_interfaces_.push_back(std::move(plugin));
  RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": started ");
}

//-----------------------------------------------------------------------------
const R2HLocalisationKFResults & R2HLocalisationFilter::get_results(const Duration & duration)
{
  // results_->setDuration(duration);
  filter_->getCurrentState(duration, results_.get());
  return *results_;
}

//-----------------------------------------------------------------------------
DiagnosticReport R2HLocalisationFilter::make_diagnostic_report(const Duration & stamp)
{
  DiagnosticReport report;
  for (auto & interface_ptr : updater_interfaces_) {
    interface_ptr->heartbeat_callback(stamp);
    report += interface_ptr->get_report();
  }

  return report;
}

}  // namespace romea
