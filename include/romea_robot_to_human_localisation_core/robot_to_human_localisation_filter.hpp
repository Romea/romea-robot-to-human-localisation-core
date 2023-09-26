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

#ifndef ROMEA_ROBOT_TO_HUMAN_LOCALISATION_CORE__ROBOT_TO_HUMAN_LOCALISATION_FILTER_HPP_
#define ROMEA_ROBOT_TO_HUMAN_LOCALISATION_CORE__ROBOT_TO_HUMAN_LOCALISATION_FILTER_HPP_

// std
#include <list>
#include <memory>
#include <string>
#include <utility>

// romea
#include <romea_core_filtering/FilterType.hpp>
#include <romea_core_filtering/kalman/KalmanFilter.hpp>
#include <romea_core_localisation/LocalisationUpdaterTwist.hpp>
#include <romea_core_localisation/LocalisationUpdaterLinearSpeed.hpp>
#include <romea_core_localisation/LocalisationUpdaterLinearSpeeds.hpp>
#include <romea_core_localisation/LocalisationUpdaterAngularSpeed.hpp>
#include <romea_core_localisation/robot_to_human/kalman/R2HLocalisationKFResults.hpp>
#include <romea_core_localisation/robot_to_human/kalman/R2HLocalisationKFPredictor.hpp>
#include <romea_core_localisation/robot_to_human/kalman/R2HLocalisationKFUpdaterRange.hpp>
#include <romea_core_localisation/robot_to_human/kalman/R2HLocalisationKFUpdaterLeaderPosition.hpp>

#include "romea_localisation_utils/filter/localisation_factory.hpp"
#include "romea_localisation_utils/filter/localisation_updater_interface.hpp"


namespace romea
{

class R2HLocalisationFilter
{
public:
  using Filter = KalmanFilter<R2HLocalisationKFMetaState, LocalisationFSMState, Duration>;
  using UpdaterTwist = LocalisationUpdaterTwist<R2HLocalisationKFMetaState>;
  using UpdaterLinearSpeed = LocalisationUpdaterLinearSpeed<R2HLocalisationKFMetaState>;
  using UpdaterLinearSpeeds = LocalisationUpdaterLinearSpeeds<R2HLocalisationKFMetaState>;
  using UpdaterAngularSpeed = LocalisationUpdaterAngularSpeed<R2HLocalisationKFMetaState>;
  using UpdaterPosition = R2HLocalisationKFUpdaterLeaderPosition;
  using UpdaterRange = R2HLocalisationKFUpdaterRange;
  using Predictor = R2HLocalisationKFPredictor;
  using Results = R2HLocalisationKFResults;

  using UpdaterInterface = LocalisationUpdaterInterfaceBase;
  using UpdaterInterfaceTwist = LocalisationUpdaterInterface<Filter, UpdaterTwist,
      romea_localisation_msgs::msg::ObservationTwist2DStamped>;
  using UpdaterInterfaceLinearSpeed = LocalisationUpdaterInterface<Filter, UpdaterLinearSpeed,
      romea_localisation_msgs::msg::ObservationTwist2DStamped>;
  using UpdaterInterfaceLinearSpeeds = LocalisationUpdaterInterface<Filter, UpdaterLinearSpeeds,
      romea_localisation_msgs::msg::ObservationTwist2DStamped>;
  using UpdaterInterfaceAngularSpeed = LocalisationUpdaterInterface<Filter, UpdaterAngularSpeed,
      romea_localisation_msgs::msg::ObservationAngularSpeedStamped>;
  using UpdaterInterfacePosition = LocalisationUpdaterInterface<Filter, UpdaterPosition,
      romea_localisation_msgs::msg::ObservationPosition2DStamped>;
  using UpdaterInterfaceRange = LocalisationUpdaterInterface<Filter, UpdaterRange,
      romea_localisation_msgs::msg::ObservationRangeStamped>;

public:
  R2HLocalisationFilter();

  explicit R2HLocalisationFilter(std::shared_ptr<rclcpp::Node> node);

  void reset();

public:
  LocalisationFSMState get_fsm_state();

  const Results & get_results(const Duration & duration);

  DiagnosticReport make_diagnostic_report(const Duration & duration);

private:
  void make_filter_(std::shared_ptr<rclcpp::Node> node);

  void make_results_(std::shared_ptr<rclcpp::Node> node);

  template<typename Interface>
  void add_proprioceptive_updater_interface_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name,
    const std::string & topic_name,
    const unsigned int & default_minimal_rate);

  void add_range_updater_interface_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name,
    const std::string & topic_name,
    const unsigned int & default_minimal_rate,
    const std::string & default_trigger_mode);


  void add_position_updater_interface_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name,
    const std::string & topic_name,
    const unsigned int & default_minimal_rate,
    const std::string & default_trigger_mode);

private:
  std::shared_ptr<Filter> filter_;
  std::unique_ptr<Results> results_;
  std::list<std::unique_ptr<UpdaterInterface>> updater_interfaces_;
};

}  // namespace romea

#endif  // ROMEA_ROBOT_TO_HUMAN_LOCALISATION_CORE__ROBOT_TO_HUMAN_LOCALISATION_FILTER_HPP_
