// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// std
#include <memory>

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_common_utils/publishers/stamped_data_publisher.hpp"
#include "romea_common_utils/conversions/pose_and_twist2d_conversions.hpp"
#include "romea_localisation_utils/conversions/localisation_status_conversions.hpp"
#include "romea_robot_to_human_localisation_core/robot_to_human_localisation.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
R2HLocalisation::R2HLocalisation(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("robot_to_human_localisation", options)),
  filter_(nullptr),
  leader_position_publisher_(nullptr),
  diagnostic_publisher_(nullptr)
{
  declare_debug(node_);
  declare_log_directory(node_);
  declare_base_footprint_frame_id(node_);
  declare_publish_rate(node_);

  make_filter_();
  make_leader_position_publisher_();
  make_diagnostic_publisher_();
  make_status_publisher_();
  make_timer_();
}

//-----------------------------------------------------------------------------
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
R2HLocalisation::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
void R2HLocalisation::make_filter_()
{
  filter_ = std::make_unique<R2HLocalisationFilter>(node_);
}

//-----------------------------------------------------------------------------
void R2HLocalisation::make_status_publisher_()
{
  using DataType = LocalisationFSMState;
  using MsgType = romea_localisation_msgs::msg::LocalisationStatus;
  status_publisher_ = make_data_publisher<DataType, MsgType>(
    node_, "status", reliable(1), true);
}


//-----------------------------------------------------------------------------
void R2HLocalisation::make_leader_position_publisher_()
{
  leader_position_publisher_ = make_stamped_data_publisher<Position2D,
      romea_common_msgs::msg::Position2DStamped>(
    node_,
    "filtered_leader_position",
    get_base_footprint_frame_id(node_),
    reliable(1),
    true);
}

//-----------------------------------------------------------------------------
void R2HLocalisation::make_diagnostic_publisher_()
{
  diagnostic_publisher_ = make_diagnostic_publisher<DiagnosticReport>(
    node_, "robot_to_human_localisation", 1.0);
}

//-----------------------------------------------------------------------------
void R2HLocalisation::make_timer_()
{
  Duration timer_period = durationFromSecond(1. / get_publish_rate(node_));
  auto callback = std::bind(&R2HLocalisation::timer_callback_, this);
  timer_ = node_->create_wall_timer(timer_period, callback);
}

//-----------------------------------------------------------------------------
void R2HLocalisation::timer_callback_()
{
  auto stamp = node_->get_clock()->now();

  LocalisationFSMState fsm_state = filter_->get_fsm_state();
  if (fsm_state == LocalisationFSMState::RUNNING) {
    const auto & results = filter_->get_results(to_romea_duration(stamp));
    Position2D leader_position = results.toLeaderPosition2D();

    std::cout << "leader position " <<
      leader_position.position.x() << " " <<
      leader_position.position.y() << " " << std::endl;

    leader_position_publisher_->publish(stamp, leader_position);
    //     rviz_.display(results);
  }

  status_publisher_->publish(fsm_state);
  publish_diagnostics_(stamp);
}

//-----------------------------------------------------------------------------
void R2HLocalisation::publish_diagnostics_(const rclcpp::Time & stamp)
{
  auto report = filter_->make_diagnostic_report(to_romea_duration(stamp));
  diagnostic_publisher_->publish(stamp, report);
}

}  // namespace romea

//-----------------------------------------------------------------------------
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::R2HLocalisation)
