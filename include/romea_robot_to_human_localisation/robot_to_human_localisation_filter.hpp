#ifndef ROMEA_ROBOT_TO_ROBOT_LOCALISATION__ROBOT_TO_ROBOT_LOCALISATION_FILTER_HPP_
#define ROMEA_ROBOT_TO_ROBOT_LOCALISATION__ROBOT_TO_ROBOT_LOCALISATION_FILTER_HPP_

// std
#include <list>
#include <memory>
#include <string>
#include <utility>

// romea
#include <romea_core_filtering/FilterType.hpp>
#include <romea_core_filtering/kalman/KalmanFilter.hpp>
#include <romea_core_localisation/robot_to_human/kalman/R2HLocalisationKFResults.hpp>
#include <romea_core_localisation/robot_to_human/kalman/R2HLocalisationKFPredictor.hpp>
#include <romea_core_localisation/robot_to_human/kalman/R2HLocalisationKFUpdaterRange.hpp>
#include <romea_core_localisation/robot_to_human/kalman/R2HLocalisationKFUpdaterLeaderPosition.hpp>
#include <romea_core_localisation/LocalisationUpdaterTwist.hpp>

#include "romea_localisation_utils/filter/localisation_factory.hpp"
#include "romea_localisation_utils/filter/localisation_updater_interface.hpp"


namespace romea
{

class R2HLocalisationFilter
{
public:
  using Filter = KalmanFilter<R2HLocalisationKFMetaState, LocalisationFSMState, Duration>;
  using UpdaterTwist = LocalisationUpdaterTwist<R2HLocalisationKFMetaState>;
  using UpdaterPosition = R2HLocalisationKFUpdaterLeaderPosition;
  using UpdaterRange = R2HLocalisationKFUpdaterRange;
  using Predictor = R2HLocalisationKFPredictor;
  using Results = R2HLocalisationKFResults;

  using UpdaterPlugin = LocalisationUpdaterInterfaceBase;
  using UpdaterPluginTwist = LocalisationUpdaterInterface<Filter, UpdaterTwist,
      romea_localisation_msgs::msg::ObservationTwist2DStamped>;
  using UpdaterPluginPosition = LocalisationUpdaterInterface<Filter, UpdaterPosition,
      romea_localisation_msgs::msg::ObservationPosition2DStamped>;
  using UpdaterPluginRange = LocalisationUpdaterInterface<Filter, UpdaterRange,
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

  void add_twist_updater_interface_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name);

  void add_range_updater_interface_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name);

  void add_position_updater_interface_(
    std::shared_ptr<rclcpp::Node> node,
    const std::string & updater_name);

private:
  std::shared_ptr<Filter> filter_;
  std::unique_ptr<Results> results_;
  std::list<std::unique_ptr<UpdaterPlugin>> updater_interfaces_;
};

}  // namespace romea

#endif  // ROMEA_ROBOT_TO_ROBOT_LOCALISATION__ROBOT_TO_ROBOT_LOCALISATION_FILTER_HPP_
