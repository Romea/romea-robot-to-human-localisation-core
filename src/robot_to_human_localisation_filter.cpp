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
  add_proprioceptive_updater_interface_<UpdaterInterfaceTwist>(node, "twist_updater");
  add_proprioceptive_updater_interface_<UpdaterInterfaceLinearSpeed>(node, "linear_speed_updater");
  add_proprioceptive_updater_interface_<UpdaterInterfaceLinearSpeeds>(
    node, "linear_speeds_updater");
  add_proprioceptive_updater_interface_<UpdaterInterfaceAngularSpeed>(
    node, "angular_speed_updater");
  add_position_updater_interface_(node, "position_updater");
  add_range_updater_interface_(node, "range_updater");
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
  declare_predictor_parameters(node);
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
  const std::string & updater_name)
{
  declare_proprioceptive_updater_parameters(node, updater_name);

  if (!get_updater_topic_name(node, updater_name).empty()) {

    using Updater = typename Interface::Updater;
    auto updater = make_proprioceptive_updater<Updater>(
      node,
      updater_name);

    auto plugin = make_updater_interface<Interface>(
      node,
      updater_name,
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
  const std::string & updater_name)
{
  declare_exteroceptive_updater_parameters(node, updater_name);
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
    updater_name,
    filter_,
    std::move(updater));

  updater_interfaces_.push_back(std::move(plugin));
  RCLCPP_INFO_STREAM(node->get_logger(), updater_name + ": started ");
}


//-----------------------------------------------------------------------------
void R2HLocalisationFilter::add_position_updater_interface_(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & updater_name)
{
  declare_exteroceptive_updater_parameters(node, updater_name);

  auto updater = make_exteroceptive_updater<UpdaterPosition, FilterType::KALMAN>(
    node,
    updater_name);

  auto plugin = make_updater_interface<UpdaterInterfacePosition>(
    node,
    updater_name,
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
