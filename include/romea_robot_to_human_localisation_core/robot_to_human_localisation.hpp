// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef ROMEA_ROBOT_TO_HUMAN_LOCALISATION_CORE__ROBOT_TO_HUMAN_LOCALISATION_HPP_
#define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_CORE__ROBOT_TO_HUMAN_LOCALISATION_HPP_

// std
#include <memory>

// romea
#include "romea_common_utils/publishers/data_publisher.hpp"
#include "romea_common_utils/conversions/position2d_conversions.hpp"
#include "romea_common_utils/publishers/diagnostic_publisher.hpp"

// local
#include "romea_robot_to_human_localisation_core/robot_to_human_localisation_filter.hpp"
#include "romea_robot_to_human_localisation_core/visibility_control.h"

namespace romea
{

class R2HLocalisation
{
public:
  ROMEA_ROBOT_TO_HUMAN_LOCALISATION_CORE_PUBLIC
  explicit R2HLocalisation(const rclcpp::NodeOptions & options);

  ROMEA_ROBOT_TO_HUMAN_LOCALISATION_CORE_PUBLIC
  virtual ~R2HLocalisation() = default;

  ROMEA_ROBOT_TO_HUMAN_LOCALISATION_CORE_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

private:
  void make_filter_();

  void make_timer_();

  void make_status_publisher_();

  void make_leader_position_publisher_();

  void make_diagnostic_publisher_();

  void publish_diagnostics_(const rclcpp::Time & stamp);

  void timer_callback_();

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::TimerBase> timer_;
  std::unique_ptr<R2HLocalisationFilter> filter_;

  std::shared_ptr<StampedPublisherBase<Position2D>> leader_position_publisher_;
  std::shared_ptr<StampedPublisherBase<DiagnosticReport>> diagnostic_publisher_;
  std::shared_ptr<PublisherBase<LocalisationFSMState>> status_publisher_;
};

}  // namespace romea

#endif  // ROMEA_ROBOT_TO_HUMAN_LOCALISATION_CORE__ROBOT_TO_HUMAN_LOCALISATION_HPP_
