/*
 * Copyright (C) 2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */
#include "lci_strategic_plugin/lci_strategic_plugin.h"
#include "lci_strategic_plugin/lci_states.h"

#define GET_MANEUVER_PROPERTY(mvr, property)                                                                           \
  (((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ?                                                 \
        (mvr).intersection_transit_left_turn_maneuver.property :                                                       \
        ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ?                                           \
             (mvr).intersection_transit_right_turn_maneuver.property :                                                 \
             ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ?                                        \
                  (mvr).intersection_transit_straight_maneuver.property :                                              \
                  ((mvr).type == cav_msgs::Maneuver::LANE_CHANGE    ? (mvr).lane_change_maneuver.property :            \
                   (mvr).type == cav_msgs::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :         \
                                                                      throw new std::invalid_argument("GET_MANEUVER_"  \
                                                                                                      "PROPERTY "      \
                                                                                                      "(property) "    \
                                                                                                      "called on "     \
                                                                                                      "maneuver with " \
                                                                                                      "invalid type "  \
                                                                                                      "id"))))))

namespace lci_strategic_plugin
{
LCIStrategicPlugin::LCIStrategicPlugin(carma_wm::WorldModelConstPtr wm, LCIStrategicPluginConfig config)
  : wm_(wm), config_(config)
{
  plugin_discovery_msg_.name = config_.strategic_plugin_name;
  plugin_discovery_msg_.versionId = "v1.0";
  plugin_discovery_msg_.available = true;
  plugin_discovery_msg_.activated = true;
  plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
  plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";

  max_comfort_accel_ = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  max_comfort_decel_ = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
  max_comfort_decel_norm_ = config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
};

cav_msgs::Plugin LCIStrategicPlugin::getDiscoveryMsg() const
{
  return plugin_discovery_msg_;
}

bool LCIStrategicPlugin::supportedLightState(lanelet::CarmaTrafficSignalState state) const
{
  switch (state)
  {
    // NOTE: Following cases are intentional fall through.
    // Supported light states
    case lanelet::CarmaTrafficSignalState::STOP_AND_REMAIN:              // Solid Red
    case lanelet::CarmaTrafficSignalState::PROTECTED_CLEARANCE:          // Yellow Solid no chance of conflicting traffic
    case lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED:   // Solid Green no chance of conflict traffic
                                                                        // (normally used with arrows)
      return true;

    // Unsupported light states
    case lanelet::CarmaTrafficSignalState::PERMISSIVE_MOVEMENT_ALLOWED:  // Solid Green there could be conflict traffic
    case lanelet::CarmaTrafficSignalState::PERMISSIVE_CLEARANCE:         // Yellow Solid there is a chance of conflicting
                                                                        // traffic
    case lanelet::CarmaTrafficSignalState::UNAVAILABLE:                  // No data available
    case lanelet::CarmaTrafficSignalState::DARK:                         // Light is non-functional
    case lanelet::CarmaTrafficSignalState::STOP_THEN_PROCEED:            // Flashing Red

    case lanelet::CarmaTrafficSignalState::PRE_MOVEMENT:                 // Yellow Red transition (Found only in the EU)
                                                                        // (normally used with arrows)
    case lanelet::CarmaTrafficSignalState::CAUTION_CONFLICTING_TRAFFIC:  // Yellow Flashing
    default:
      return false;
  }
}

LCIStrategicPlugin::VehicleState LCIStrategicPlugin::extractInitialState(const cav_srvs::PlanManeuversRequest& req) const
{
  VehicleState state;
  if (!req.prior_plan.maneuvers.empty())
  {
    ROS_DEBUG_STREAM("Provided with initial plan...");
    state.stamp = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_time);
    state.downtrack = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_dist);
    state.speed = GET_MANEUVER_PROPERTY(req.prior_plan.maneuvers.back(), end_speed);
    state.lane_id = getLaneletsBetweenWithException(state.downtrack, state.downtrack, true).front().id();
  }
  else
  {
    ROS_DEBUG_STREAM("No initial plan provided...");
    
    state.stamp = req.header.stamp;
    state.downtrack = req.veh_downtrack;
    state.speed = req.veh_logitudinal_velocity;
    state.lane_id = stoi(req.veh_lane_id);
  }
  ROS_DEBUG_STREAM("extractInitialState >>>> state.stamp: " << state.stamp);
  ROS_DEBUG_STREAM("extractInitialState >>>> state.downtrack : " << state.downtrack );
  ROS_DEBUG_STREAM("extractInitialState >>>> state.speed: " << state.speed);
  ROS_DEBUG_STREAM("extractInitialState >>>> state.lane_id: " << state.lane_id);

  return state;
}

bool LCIStrategicPlugin::validLightState(const boost::optional<std::pair<boost::posix_time::ptime, lanelet::CarmaTrafficSignalState>>& optional_state,
                                        const ros::Time& source_time) const
{
  if (!optional_state)
  {
    ROS_WARN_STREAM("Traffic light data not available for source_time " << std::to_string(source_time.toSec()));
    return false;
  }

  lanelet::CarmaTrafficSignalState light_state = optional_state.get().second;

  if (!supportedLightState(light_state))
  {
    ROS_ERROR_STREAM("LCIStrategic Plugin asked to handle CarmaTrafficSignalState: " << light_state
                                                                                << " which is not supported.");
    return false;
  }

  return true;
}

std::vector<lanelet::ConstLanelet> LCIStrategicPlugin::getLaneletsBetweenWithException(double start_downtrack,
                                                                                      double end_downtrack,
                                                                                      bool shortest_path_only,
                                                                                      bool bounds_inclusive) const
{
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
      wm_->getLaneletsBetween(start_downtrack, end_downtrack, shortest_path_only, bounds_inclusive);

  if (crossed_lanelets.size() == 0)
  {
    throw std::invalid_argument("getLaneletsBetweenWithException called but inputs do not cross any lanelets going "
                                "from: " +
                                std::to_string(start_downtrack) + " to: " + std::to_string(end_downtrack));
  }

  return crossed_lanelets;
}

void LCIStrategicPlugin::planWhenUNAVAILABLE(const cav_srvs::PlanManeuversRequest& req,
                                            cav_srvs::PlanManeuversResponse& resp, const VehicleState& current_state,
                                            const lanelet::CarmaTrafficSignalPtr& traffic_light, const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, const lanelet::ConstLanelet& current_lanelet)
{
  // Reset intersection state since in this state we are not yet known to be in or approaching an intersection
  intersection_speed_ = boost::none;
  intersection_end_downtrack_ = boost::none;

  if (!traffic_light)
  {
    ROS_DEBUG("No lights found along route. Returning maneuver plan unchanged");
    return;
  }

  double max_comfort_accel = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  double max_comfort_decel = -1.0 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;

  auto stop_line = traffic_light->getStopLine(entry_lanelet);

  if (!stop_line)
  {
    throw std::invalid_argument("Given entry lanelet doesn't have stop_line...");
  }

  double traffic_light_down_track =
      wm_->routeTrackPos(stop_line.get().front().basicPoint2d()).downtrack;

  ROS_DEBUG("traffic_light_down_track %f", traffic_light_down_track);

  double distance_remaining_to_traffic_light = traffic_light_down_track - current_state.downtrack - config_.vehicle_length;

  ROS_DEBUG_STREAM("distance_remaining_to_traffic_light: " << distance_remaining_to_traffic_light <<
                    ", current_state.downtrack: " << current_state.downtrack <<
                    ", config_.vehicle_length: " << config_.vehicle_length);

  intersection_speed_ = findSpeedLimit(traffic_light->getControlStartLanelets().front());

  ROS_DEBUG_STREAM("intersection_speed_: " << intersection_speed_.get());

  auto speed_limit = findSpeedLimit(current_lanelet);

  ROS_DEBUG_STREAM("speed_limit (free flow speed): " << speed_limit);

  double time_remaining_at_free_flow =
      (2.0 * (distance_remaining_to_traffic_light - config_.vehicle_length)) /
      (intersection_speed_.get() + current_state.speed);  // Kinematic Equation: 2*d / (vf + vi) = t

  double trajectory_smoothing_activation_distance = get_trajectory_smoothing_activation_distance(time_remaining_at_free_flow, lanelet::time::toSec(traffic_light->fixed_cycle_duration), 
                                                                                      current_state.speed, speed_limit, intersection_speed_.get(), max_comfort_accel, max_comfort_decel);
  if (trajectory_smoothing_activation_distance < 0)
    trajectory_smoothing_activation_distance = distance_remaining_to_traffic_light;
    
  ROS_DEBUG_STREAM("trajectory_smoothing_activation_distance: " << trajectory_smoothing_activation_distance);

  double stopping_dist = estimate_distance_to_stop(current_state.speed, config_.vehicle_decel_limit_multiplier  *
                                                                            config_.vehicle_decel_limit); //accepts positive decel

  ROS_DEBUG_STREAM("Stopping distance: " << stopping_dist);

  double plugin_activation_distance = std::max(stopping_dist, config_.min_approach_distance);

  plugin_activation_distance = std::max(plugin_activation_distance, trajectory_smoothing_activation_distance);

  ROS_DEBUG_STREAM("plugin_activation_distance: " << plugin_activation_distance);

  if (distance_remaining_to_traffic_light <= plugin_activation_distance)
  {
    ROS_INFO_STREAM("Within intersection range");
    transition_table_.signal(TransitEvent::IN_STOPPING_RANGE);  // Evaluate Signal and Run Trajectory Smoothing Algorithm
  }
  else
  {
    ROS_DEBUG_STREAM("Not within intersection range");
  }
}


boost::optional<bool> LCIStrategicPlugin::canArriveAtGreenWithCertainty(const ros::Time& light_arrival_time_by_algo, const lanelet::CarmaTrafficSignalPtr& traffic_light) const
{
    ros::Time early_arrival_time_by_algo =
        light_arrival_time_by_algo - ros::Duration(config_.green_light_time_buffer);

    ros::Time late_arrival_time_by_algo =
        light_arrival_time_by_algo + ros::Duration(config_.green_light_time_buffer);

    ROS_DEBUG_STREAM("light_arrival_time_by_algo: " << std::to_string(light_arrival_time_by_algo.toSec()));
    ROS_DEBUG_STREAM("early_arrival_time_by_algo: " << std::to_string(early_arrival_time_by_algo.toSec()));
    ROS_DEBUG_STREAM("late_arrival_time_by_algo: " << std::to_string(late_arrival_time_by_algo.toSec()));

    auto early_arrival_state_by_algo_optional = traffic_light->predictState(lanelet::time::timeFromSec(early_arrival_time_by_algo.toSec()));

    if (!validLightState(early_arrival_state_by_algo_optional, early_arrival_time_by_algo))
      return boost::none;

    ROS_DEBUG_STREAM("early_arrival_state_by_algo: " << early_arrival_state_by_algo_optional.get().second);

    auto late_arrival_state_by_algo_optional = traffic_light->predictState(lanelet::time::timeFromSec(late_arrival_time_by_algo.toSec()));

    if (!validLightState(late_arrival_state_by_algo_optional, late_arrival_time_by_algo))
      return boost::none; 

    ROS_DEBUG_STREAM("late_arrival_state_by_algo: " << late_arrival_state_by_algo_optional.get().second);

    // We will cross the light on the green phase even if we arrive early or late
    if (early_arrival_state_by_algo_optional.get().second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED &&
        late_arrival_state_by_algo_optional.get().second ==
            lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED)  // Green light
      return true;
    else
      return false;

}

void LCIStrategicPlugin::handleStopping(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp, 
                                        const VehicleState& current_state, 
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light,
                                        const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, const lanelet::ConstLanelet& current_lanelet,
                                        const ros::Time& nearest_green_entry_time,
                                        double traffic_light_down_track)
{
  double distance_remaining_to_traffic_light = traffic_light_down_track - current_state.downtrack;
  
  // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
        getLaneletsBetweenWithException(current_state.downtrack, traffic_light_down_track, true, true);

  double safe_distance_to_stop = pow(current_state.speed, 2)/(2 * max_comfort_decel_norm_) + config_.min_gap;
  ROS_DEBUG_STREAM("safe_distance_to_stop at max_comfort_decel:  " << safe_distance_to_stop << ", max_comfort_decel_norm_: " << max_comfort_decel_norm_);
  
  // earlier stop than this would result in stopping way before intersection
  double min_bound_stop_time =
    (2.0 * (distance_remaining_to_traffic_light - config_.vehicle_length)) / current_state.speed;  // Kinematic Equation: 2*d / (vf + vi) = t where vf = 0

  ROS_DEBUG_STREAM("min_bound_stop_time in sec:  " << min_bound_stop_time);

  if (safe_distance_to_stop < distance_remaining_to_traffic_light)
  {
    auto state_pair_at_stop = traffic_light->predictState(lanelet::time::timeFromSec(current_state.stamp.toSec() + min_bound_stop_time));
    
    if (!validLightState(state_pair_at_stop, current_state.stamp + ros::Duration(min_bound_stop_time)))
    return;

    ROS_ERROR_STREAM("Checking STOPPING state: " << state_pair_at_stop.get().second << ", at predicted time: " << current_state.stamp.toSec() + min_bound_stop_time);

    // perfectly stop at red/yellow with given distance and constant deceleration 
    if (state_pair_at_stop.get().second != lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED)
    {
      double decel_rate = current_state.speed / min_bound_stop_time; // Kinematic |(v_f - v_i) / t = a|
      ROS_DEBUG_STREAM("Planning stop and wait maneuver at decel_rate: -" << decel_rate);
      
      resp.new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(
        current_state.downtrack, traffic_light_down_track - config_.vehicle_length, current_state.speed, crossed_lanelets.front().id(),
        crossed_lanelets.back().id(), current_state.stamp,
        req.header.stamp + ros::Duration(config_.min_maneuver_planning_period), decel_rate));
      return;
    }

    // 2. If stopping with single deceleration falls on green phase, stop at next possible red phase
    //    using 2 different decel_rate (max_decel then custom_decel)
    
    // attempt to find suitable red/yellow time to stop
    auto predicted_state_pair = traffic_light->predictState(lanelet::time::timeFromSec(current_state.stamp.toSec() + min_bound_stop_time));
    bool new_stop_time_found = false;

    double max_stop_time_stamp = nearest_green_entry_time.toSec();
    double target_stop_time = min_bound_stop_time;
    ROS_ERROR_STREAM("Checking INITIAL state: " << predicted_state_pair.get().second << ", with end_time: " << predicted_state_pair.get().first 
                                                                                     << ", at predicted_time: " << current_state.stamp.toSec() + min_bound_stop_time);
    
    while (lanelet::time::toSec(predicted_state_pair.get().first) < max_stop_time_stamp + lanelet::time::toSec(traffic_light->fixed_cycle_duration)) // look for valid stop light within 1 cycle
    {
      if (predicted_state_pair.get().second != lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED)
      {
        // check if midway timestamp of this signal state
        double stop_time_temp = lanelet::time::toSec(predicted_state_pair.get().first)
                                  - lanelet::time::toSec(traffic_light->signal_durations[predicted_state_pair.get().second])/2
                                  - current_state.stamp.toSec();
        ROS_DEBUG_STREAM("Found new stop time at: " << std::to_string(stop_time_temp) << ", where min_bound_stop_time is: " << std::to_string(min_bound_stop_time));
        target_stop_time = std::max(stop_time_temp, min_bound_stop_time); // min_bound_stop is guaranteed to be on same signal state
        new_stop_time_found = true;
        break;
      }
      predicted_state_pair = traffic_light->predictState(predicted_state_pair.get().first + boost::posix_time::milliseconds(200)); //get next state
      ROS_ERROR_STREAM("Checking state: " << predicted_state_pair.get().second << ", at end_time: " << predicted_state_pair.get().first);
    }

    // Stopping by decelerating at max_decel some distance and then stopping with custom decel_rate for total of target_stop_time
    if (new_stop_time_found)
    {
      // try calculating speed to decelerate before calling stop_and_wait
      double speed_before_stop = (std::pow(current_state.speed, 2) + 2 * distance_remaining_to_traffic_light * max_comfort_decel_ ) / (current_state.speed + target_stop_time * max_comfort_decel_);
      
      ROS_DEBUG_STREAM("New RED StopTime's speed before calling stop_and_wait_plugin: " << speed_before_stop << ", at decel_rate: " << max_comfort_decel_);
      
      if (speed_before_stop < current_state.speed || speed_before_stop > 0) //todo change 0 to minimum speed?
      {
        // calculate necessary parameters
        double decelerating_time = (current_state.speed - speed_before_stop) / max_comfort_decel_norm_;
        double stopping_time = target_stop_time - decelerating_time;
        double stopping_accel_norm = speed_before_stop / stopping_time;
        double stopping_distance = pow(speed_before_stop, 2)/(2 * stopping_accel_norm) + config_.min_gap;
        double start_stopping_downtrack = traffic_light_down_track - stopping_distance;

        ros::Time stop_starting_timestamp = current_state.stamp + ros::Duration(decelerating_time);
        ROS_DEBUG_STREAM("Found stop_starting_timestamp at: " << std::to_string(stop_starting_timestamp.toSec()) << ", where current_state is: " << std::to_string(current_state.stamp.toSec()));

        lanelet::ConstLanelet starting_lanelet_for_stop = crossed_lanelets.front();
        for (auto llt: crossed_lanelets)
        {
          if (wm_->routeTrackPos(llt.centerline2d().back().basicPoint2d()).downtrack < current_state.downtrack + stopping_distance)
          {
            starting_lanelet_for_stop = llt;
          }
        }
        ROS_DEBUG_STREAM("Modified Stop: speed_before_stop at: " << speed_before_stop << ", stopping_accel_norm: " << stopping_accel_norm);
        ROS_DEBUG_STREAM("Modified Stop: stopping_distance at: " << stopping_distance << ", start_stopping_downtrack: " << start_stopping_downtrack);

        // compose ts_params manually with above parameters
        TrajectorySmoothingParameters ts_params;
        ts_params.a_decel = max_comfort_decel_;
        ts_params.case_num = SpeedProfileCase::DECEL_ACCEL;
        ts_params.dist_accel = 0.0;
        ts_params.dist_cruise = 0.0;
        ts_params.dist_decel = start_stopping_downtrack - current_state.downtrack;
        ts_params.is_algorithm_successful = true;
        ts_params.speed_before_decel = current_state.speed;

        // first decelerate (triggering algorithm as if the scheduled entry and target speed is at start of stopping downtrack) 
        resp.new_plan.maneuvers.push_back(composeTrajectorySmoothingManeuverMessage(current_state.downtrack, start_stopping_downtrack, 
                                            current_state.speed, speed_before_stop, current_state.stamp, stop_starting_timestamp, ts_params));
        
        // then stop
        resp.new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(
            start_stopping_downtrack, traffic_light_down_track - config_.vehicle_length, speed_before_stop,
            starting_lanelet_for_stop.id(), exit_lanelet.id(), stop_starting_timestamp,
            stop_starting_timestamp + ros::Duration(config_.min_maneuver_planning_period), stopping_accel_norm));
        
        return;
      }
    } 
  }
}
void LCIStrategicPlugin::planWhenAPPROACHING(const cav_srvs::PlanManeuversRequest& req,
                                            cav_srvs::PlanManeuversResponse& resp, const VehicleState& current_state,
                                            const lanelet::CarmaTrafficSignalPtr& traffic_light, const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, const lanelet::ConstLanelet& current_lanelet)
{
  if (!traffic_light)  // If we are in the approaching state and there is no longer any lights ahead of us then
                       // the vehicle must have crossed the stop bar
  {
    transition_table_.signal(TransitEvent::CROSSED_STOP_BAR);
    return;
  }

  auto stop_line = traffic_light->getStopLine(entry_lanelet);

  if (!stop_line)
  {
    throw std::invalid_argument("Given entry lanelet doesn't have stop_line...");
  }

  double traffic_light_down_track =
      wm_->routeTrackPos(stop_line.get().front().basicPoint2d()).downtrack;

  ROS_DEBUG("traffic_light_down_track %f", traffic_light_down_track);

  double distance_remaining_to_traffic_light = traffic_light_down_track - current_state.downtrack;

  // If the vehicle is at a stop trigger the stopped state
  constexpr double HALF_MPH_IN_MPS = 0.22352;
  if (current_state.speed < HALF_MPH_IN_MPS &&
      fabs(distance_remaining_to_traffic_light) < config_.stopping_location_buffer)
  {
    transition_table_.signal(TransitEvent::STOPPED);  // The vehicle has come to a stop at the light
    return;
  }

  // At this point we know the vehicle is within the activation distance and we know the current and next light phases
  // All we need to now determine is if we should stop or if we should continue

  intersection_speed_ = findSpeedLimit(entry_lanelet); //technically lanelet is not "inside the lanelet"

  double speed_limit = findSpeedLimit(current_lanelet);

  ROS_DEBUG_STREAM("intersection_speed_: " << intersection_speed_.get());

  intersection_end_downtrack_ =
      wm_->routeTrackPos(exit_lanelet.centerline2d().front().basicPoint2d()).downtrack;

  ROS_DEBUG_STREAM("intersection_end_downtrack_: " << intersection_end_downtrack_.get());

  // Start of TSMO UC2 Algorithm

  ros::Time earliest_entry_time = current_state.stamp + get_earliest_entry_time(distance_remaining_to_traffic_light, speed_limit, 
                                                  current_state.speed, intersection_speed_.get(), max_comfort_accel_, max_comfort_decel_);

  ROS_DEBUG_STREAM("earliest_entry_time: " << std::to_string(earliest_entry_time.toSec()) << ", with : " << earliest_entry_time - current_state.stamp  << " left at: " << std::to_string(current_state.stamp.toSec()));

  ros::Time nearest_green_entry_time = get_nearest_green_entry_time(current_state.stamp, earliest_entry_time, traffic_light) 
                                          + ros::Duration(config_.green_light_time_buffer + 0.2); //0.2sec more buffer since green_light buffer also ends at previous state
                                                                                                  // TODO need review for above

  ROS_DEBUG_STREAM("nearest_green_entry_time with buffer: " << std::to_string(nearest_green_entry_time.toSec()));

  // CASE SELECTION START
  TrajectorySmoothingParameters ts_params;

  double estimated_entry_time = calc_estimated_entry_time_left(distance_remaining_to_traffic_light, current_state.speed, intersection_speed_.get() );
  double remaining_time = nearest_green_entry_time.toSec() - current_state.stamp.toSec();
  double speed_before_decel = calc_speed_before_decel(remaining_time, distance_remaining_to_traffic_light, current_state.speed, intersection_speed_.get());
  double speed_before_accel = calc_speed_before_accel(remaining_time, distance_remaining_to_traffic_light, current_state.speed, intersection_speed_.get());
  SpeedProfileCase case_num = determine_speed_profile_case(estimated_entry_time, remaining_time, speed_before_decel, speed_before_accel, speed_limit);
  ROS_WARN_STREAM("ALERT: case_num: " << (int)case_num);
  // change speed profile depending on algorithm case starting from maneuver start_dist
  if(case_num == ACCEL_CRUISE_DECEL || case_num == ACCEL_DECEL){
    // acceleration (cruising if needed) then deceleration to reach desired intersection entry speed/time according to algorithm doc
    ts_params = get_parameters_for_accel_cruise_decel_speed_profile(distance_remaining_to_traffic_light, remaining_time, current_state.speed, speed_before_decel, speed_limit, intersection_speed_.get());
  }
  else if(case_num == DECEL_ACCEL || case_num == DECEL_CRUISE_ACCEL)
  {
    // deceleration (cruising if needed) then acceleration to reach desired intersection entry speed/time according to algorithm doc
    ts_params = get_parameters_for_decel_cruise_accel_speed_profile(distance_remaining_to_traffic_light, remaining_time, current_state.speed, speed_before_accel, config_.minimum_speed, intersection_speed_.get());
  }
  else
  {
    throw std::invalid_argument("The light controlled intersection tactical plugin doesn't handle the case number requested");
  }

  ts_params.case_num = case_num;

  // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
        getLaneletsBetweenWithException(current_state.downtrack, traffic_light_down_track, true, true);

  // CASE SELECTION END

  // Although algorithm determines nearest_green_time is possible, check if the vehicle can arrive with certainty
  if (ts_params.is_algorithm_successful) 
  {
    ros::Time light_arrival_time_by_algo = current_state.stamp + ros::Duration(remaining_time);
    auto can_make_green_optional = canArriveAtGreenWithCertainty(light_arrival_time_by_algo, traffic_light);
    
    // no change for maneuver if invalid light states
    if (!can_make_green_optional) 
      return;
    
    if (can_make_green_optional.get())
    {
      ROS_DEBUG_STREAM("Algorithm successful, and able to make it at green with certainty. Planning traj smooth and intersection transit maneuvers");
      
      resp.new_plan.maneuvers.push_back(composeTrajectorySmoothingManeuverMessage(current_state.downtrack, traffic_light_down_track - config_.vehicle_length, 
                                            current_state.speed, intersection_speed_.get(), current_state.stamp, light_arrival_time_by_algo, ts_params));

      double intersection_length = intersection_end_downtrack_.get() - traffic_light_down_track;

      ros::Time intersection_exit_time =
          light_arrival_time_by_algo + ros::Duration(intersection_length / intersection_speed_.get());

      resp.new_plan.maneuvers.push_back(composeIntersectionTransitMessage(
          traffic_light_down_track - config_.vehicle_length, intersection_end_downtrack_.get(), intersection_speed_.get(),
          intersection_speed_.get(), light_arrival_time_by_algo, intersection_exit_time, entry_lanelet.id(), exit_lanelet.id()));
      return;
    }
  }

  // if algorithm is NOT successful or if the vehicle cannot make the green light
  // 1. Try to stop with max_decel and see if it aligns with red/yellow. If it does, go ahead and stop
  // 2. If stopping with single deceleration falls on green phase, stop at next possible red phase by using 2 different decel_rate (max_decel then custom_decel)
  handleStopping(req,resp, current_state, traffic_light, entry_lanelet, exit_lanelet, current_lanelet, nearest_green_entry_time, traffic_light_down_track);

  if (!resp.new_plan.maneuvers.empty()) // able to stop
    return;

  // 3. if not able to stop nor reach target speed at green, attempt its best to reach the target parameters at the intersection
  // ts_params would have been modified to try accommodate this scenario
  ROS_DEBUG_STREAM("The vehicle is not able to stop at red/yellow light nor is able to reach target speed at green. Attempting its best to pass through at green!");
  resp.new_plan.maneuvers.push_back(composeTrajectorySmoothingManeuverMessage(current_state.downtrack, traffic_light_down_track - config_.vehicle_length, 
                                          current_state.speed, intersection_speed_.get(), current_state.stamp, nearest_green_entry_time, ts_params));

  double intersection_length = intersection_end_downtrack_.get() - traffic_light_down_track;

  ros::Time intersection_exit_time =
      nearest_green_entry_time + ros::Duration(intersection_length / intersection_speed_.get());

  resp.new_plan.maneuvers.push_back(composeIntersectionTransitMessage(
      traffic_light_down_track - config_.vehicle_length, intersection_end_downtrack_.get(), intersection_speed_.get(),
      intersection_speed_.get(), nearest_green_entry_time, intersection_exit_time, crossed_lanelets.back().id(), exit_lanelet.id()));

}

void LCIStrategicPlugin::planWhenWAITING(const cav_srvs::PlanManeuversRequest& req,
                                        cav_srvs::PlanManeuversResponse& resp, const VehicleState& current_state,
                                        const lanelet::CarmaTrafficSignalPtr& traffic_light, const lanelet::ConstLanelet& entry_lanelet, const lanelet::ConstLanelet& exit_lanelet, const lanelet::ConstLanelet& current_lanelet)
{
  if (!traffic_light)
  {
    throw std::invalid_argument("While in WAITING state, the traffic lights disappeared.");
  }

  double traffic_light_down_track =
      wm_->routeTrackPos(traffic_light->stopLine().front().front().basicPoint2d()).downtrack;

  ROS_DEBUG("traffic_light_down_track %f", traffic_light_down_track);

  auto current_light_state_optional = traffic_light->predictState(lanelet::time::timeFromSec(req.header.stamp.toSec()));

  if (!validLightState(current_light_state_optional, req.header.stamp))
    return;

  if (current_light_state_optional.get().second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED)
  {
    transition_table_.signal(TransitEvent::RED_TO_GREEN_LIGHT);  // If the light is green send the light transition
                                                                 // signal
    return;
  }

  // A fixed buffer to add to stopping maneuvers once the vehicle is already stopped to ensure that they can have their
  // trajectory planned
  constexpr double stop_maneuver_buffer = 10.0;

  // If the light is not green then continue waiting by creating a stop and wait maneuver on top of the vehicle
  double stopping_accel = config_.vehicle_decel_limit_multiplier * config_.vehicle_decel_limit;

  resp.new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(
      current_state.downtrack - stop_maneuver_buffer, traffic_light_down_track, current_state.speed,
      current_state.lane_id, current_state.lane_id, current_state.stamp,
      current_state.stamp + ros::Duration(config_.min_maneuver_planning_period), stopping_accel));
}

void LCIStrategicPlugin::planWhenDEPARTING(const cav_srvs::PlanManeuversRequest& req,
                                          cav_srvs::PlanManeuversResponse& resp, const VehicleState& current_state,
                                          double intersection_end_downtrack, double intersection_speed_limit)
{
  if (current_state.downtrack > intersection_end_downtrack)
  {
    transition_table_.signal(TransitEvent::INTERSECTION_EXIT);  // If we are past the most recent
    return;
  }

  // Calculate exit time assuming constant acceleration
  ros::Time intersection_exit_time =
      current_state.stamp +
      ros::Duration(2.0 * (intersection_end_downtrack - current_state.downtrack) / (current_state.speed + intersection_speed_limit));

  // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
      getLaneletsBetweenWithException(current_state.downtrack, intersection_end_downtrack, true, true);

  // Compose intersection transit maneuver
  resp.new_plan.maneuvers.push_back(composeIntersectionTransitMessage(
      current_state.downtrack, intersection_end_downtrack, current_state.speed, intersection_speed_limit,
      current_state.stamp, intersection_exit_time, crossed_lanelets.front().id(), crossed_lanelets.back().id()));
}

bool LCIStrategicPlugin::planManeuverCb(cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp)
{
  if (!wm_->getRoute())
  {
    ROS_ERROR_STREAM("Could not plan maneuvers as route was not available");
    return true;
  }

  ROS_DEBUG("Finding car information");

  // Extract vehicle data from request
  VehicleState current_state = extractInitialState(req);
  if (transition_table_.getState() != TransitState::UNAVAILABLE && !req.prior_plan.maneuvers.empty())
  {
    ROS_WARN_STREAM("State is NOT UNAVAILABLE AND Maneuvers in request is NOT empty");
    return true;
  }
  // Get current traffic light information
  ROS_DEBUG("\n\nFinding traffic_light information");

  auto traffic_list = wm_->getSignalsAlongRoute({ req.veh_x, req.veh_y });
  ROS_DEBUG_STREAM("Found traffic lights of size: " << traffic_list.size());
  auto current_lanelets = wm_->getLaneletsFromPoint({ req.veh_x, req.veh_y});
  lanelet::ConstLanelet current_lanelet;
  
  if (current_lanelets.empty())
  {
    ROS_ERROR_STREAM("Given vehicle position is not on the road! Returning...");
    return true;
  }

  // get the lanelet that is on the route in case overlapping ones found
  for (auto llt : current_lanelets)
  {
    auto route = wm_->getRoute()->shortestPath();
    if (std::find(route.begin(), route.end(), llt) != route.end())
    {
      current_lanelet = llt;
      break;
    }
  }

  lanelet::CarmaTrafficSignalPtr nearest_traffic_signal = nullptr;
  
  lanelet::ConstLanelet entry_lanelet;
  lanelet::ConstLanelet exit_lanelet;

  for (auto signal : traffic_list)
  {
    auto optional_entry_exit = wm_->getEntryExitOfSignalAlongRoute(signal);
    // if signal is not matching our destination skip
    if (!optional_entry_exit)
    {
      ROS_ERROR_STREAM("Did not find entry.exit along the route");
      continue;
    }
      
    entry_lanelet = optional_entry_exit.get().first;
    exit_lanelet = optional_entry_exit.get().second;

    nearest_traffic_signal = signal;
    break;
  }
  
  TransitState prev_state;

  do
  {
    // Clear previous maneuvers planned by this plugin as guard against state change since each state generates an
    // independent set of maneuvers
    resp.new_plan = cav_msgs::ManeuverPlan();

    prev_state = transition_table_.getState();  // Cache previous state to check if state has changed after 1 iteration

    /* NOTE: Leaving this commented out code is intentional to provide an easy way to monitor light state at runtime. If a better way is implemented then this can be removed
    if (!traffic_list.empty()) { 
      auto traffic_light = traffic_list.front();
      ROS_ERROR_STREAM("\n\nCurrent Light State: " << traffic_light->getState().get() 
      << std::endl <<  "                      1: " << traffic_light->predictState(lanelet::time::timeFromSec((ros::Time::now() + ros::Duration(1.0)).toSec())).get()
      << std::endl <<  "                      2: " << traffic_light->predictState(lanelet::time::timeFromSec((ros::Time::now() + ros::Duration(2.0)).toSec())).get()
      << std::endl <<  "                      3: " << traffic_light->predictState(lanelet::time::timeFromSec((ros::Time::now() + ros::Duration(3.0)).toSec())).get()
      << std::endl <<  "                      4: " << traffic_light->predictState(lanelet::time::timeFromSec((ros::Time::now() + ros::Duration(4.0)).toSec())).get()
      << std::endl <<  "                      5: " << traffic_light->predictState(lanelet::time::timeFromSec((ros::Time::now() + ros::Duration(5.0)).toSec())).get()
      << std::endl);
    }
    */
    ROS_INFO_STREAM("Planning in state: " << transition_table_.getState());
    switch (transition_table_.getState())
    {
      case TransitState::UNAVAILABLE:
        planWhenUNAVAILABLE(req, resp, current_state, nearest_traffic_signal, entry_lanelet, exit_lanelet, current_lanelet);
        break;

      case TransitState::APPROACHING:
        planWhenAPPROACHING(req, resp, current_state, nearest_traffic_signal, entry_lanelet, exit_lanelet, current_lanelet);
        break;

      case TransitState::WAITING:
        planWhenWAITING(req, resp, current_state, nearest_traffic_signal, entry_lanelet, exit_lanelet, current_lanelet);
        break;

      case TransitState::DEPARTING:
        // Reset intersection state since we are no longer in an intersection
        if (!intersection_end_downtrack_ || !intersection_speed_)
        {
          throw std::invalid_argument("State is DEPARTING but the intersection variables are not available");
        }
        planWhenDEPARTING(req, resp, current_state, intersection_end_downtrack_.get(), intersection_speed_.get());
        break;

      default:
        throw std::invalid_argument("LCIStrategic Strategic Plugin entered unknown state");
        break;
    }

  } while (transition_table_.getState() != prev_state);  // If the state has changed then continue looping

  return true;
  // We need to evaluate the events so the state transitions can be triggered
}

cav_msgs::Maneuver LCIStrategicPlugin::composeTrajectorySmoothingManeuverMessage(double start_dist, double end_dist, double start_speed,
                                                       double target_speed, ros::Time start_time, ros::Time end_time,
                                                       const TrajectorySmoothingParameters& tsp) const
{
  cav_msgs::Maneuver maneuver_msg;
  maneuver_msg.type = cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.negotiation_type =
      cav_msgs::ManeuverParameters::NO_NEGOTIATION;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.presence_vector =
      cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.planning_tactical_plugin =
      config_.lane_following_plugin_name; // since maneuver is technically before intersection
  maneuver_msg.intersection_transit_straight_maneuver.parameters.planning_strategic_plugin =
      config_.strategic_plugin_name;
  maneuver_msg.intersection_transit_straight_maneuver.start_dist = start_dist;
  maneuver_msg.intersection_transit_straight_maneuver.start_speed = start_speed;
  maneuver_msg.intersection_transit_straight_maneuver.start_time = start_time;
  maneuver_msg.intersection_transit_straight_maneuver.end_dist = end_dist;
  maneuver_msg.intersection_transit_straight_maneuver.end_speed = target_speed;
  maneuver_msg.intersection_transit_straight_maneuver.end_time = end_time;

  maneuver_msg.intersection_transit_straight_maneuver.parameters.string_valued_meta_data.push_back(light_controlled_intersection_strategy_);
  maneuver_msg.intersection_transit_straight_maneuver.parameters.float_valued_meta_data.push_back(tsp.a_accel);
  maneuver_msg.intersection_transit_straight_maneuver.parameters.float_valued_meta_data.push_back(tsp.a_decel);
  maneuver_msg.intersection_transit_straight_maneuver.parameters.float_valued_meta_data.push_back(tsp.dist_accel);
  maneuver_msg.intersection_transit_straight_maneuver.parameters.float_valued_meta_data.push_back(tsp.dist_cruise);
  maneuver_msg.intersection_transit_straight_maneuver.parameters.float_valued_meta_data.push_back(tsp.dist_decel);
  maneuver_msg.intersection_transit_straight_maneuver.parameters.float_valued_meta_data.push_back(tsp.speed_before_accel);
  maneuver_msg.intersection_transit_straight_maneuver.parameters.float_valued_meta_data.push_back(tsp.speed_before_decel);
  maneuver_msg.intersection_transit_straight_maneuver.parameters.int_valued_meta_data.push_back(static_cast<int>(tsp.case_num));

  // Start time and end time for maneuver are assigned in updateTimeProgress

  ROS_INFO_STREAM("Creating TrajectorySmoothingManeuver start dist: " << start_dist << " end dist: " << end_dist
                                                                      << " start_time: " << start_time.toSec()
                                                                      << " end_time: " << end_time.toSec());

  return maneuver_msg;
}

cav_msgs::Maneuver LCIStrategicPlugin::composeStopAndWaitManeuverMessage(double current_dist, double end_dist,
                                                                        double start_speed,
                                                                        const lanelet::Id& starting_lane_id,
                                                                        const lanelet::Id& ending_lane_id,
                                                                        ros::Time start_time, ros::Time end_time, double stopping_accel) const
{
  cav_msgs::Maneuver maneuver_msg;
  maneuver_msg.type = cav_msgs::Maneuver::STOP_AND_WAIT;
  maneuver_msg.stop_and_wait_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
  maneuver_msg.stop_and_wait_maneuver.parameters.presence_vector =
      cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN | cav_msgs::ManeuverParameters::HAS_FLOAT_META_DATA;
  maneuver_msg.stop_and_wait_maneuver.parameters.planning_tactical_plugin = config_.stop_and_wait_plugin_name;
  maneuver_msg.stop_and_wait_maneuver.parameters.planning_strategic_plugin = config_.strategic_plugin_name;
  maneuver_msg.stop_and_wait_maneuver.start_dist = current_dist;
  maneuver_msg.stop_and_wait_maneuver.end_dist = end_dist;
  maneuver_msg.stop_and_wait_maneuver.start_speed = start_speed;
  maneuver_msg.stop_and_wait_maneuver.start_time = start_time;
  maneuver_msg.stop_and_wait_maneuver.end_time = end_time;
  maneuver_msg.stop_and_wait_maneuver.starting_lane_id = std::to_string(starting_lane_id);
  maneuver_msg.stop_and_wait_maneuver.ending_lane_id = std::to_string(ending_lane_id);

  // Set the meta data for the stop location buffer
  maneuver_msg.stop_and_wait_maneuver.parameters.float_valued_meta_data.push_back(config_.stopping_location_buffer);
  maneuver_msg.stop_and_wait_maneuver.parameters.float_valued_meta_data.push_back(stopping_accel);

  ROS_INFO_STREAM("Creating stop and wait start dist: " << current_dist << " end dist: " << end_dist
                                                                        << " start_time: " << start_time.toSec()
                                                                        << " end_time: " << end_time.toSec()
                                                                        << " stopping_accel: " << stopping_accel);

  return maneuver_msg;
}

cav_msgs::Maneuver LCIStrategicPlugin::composeIntersectionTransitMessage(double start_dist, double end_dist,
                                                                        double start_speed, double target_speed,
                                                                        ros::Time start_time, ros::Time end_time,
                                                                        const lanelet::Id& starting_lane_id,
                                                                        const lanelet::Id& ending_lane_id) const
{
  cav_msgs::Maneuver maneuver_msg;
  maneuver_msg.type = cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.negotiation_type =
      cav_msgs::ManeuverParameters::NO_NEGOTIATION;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.presence_vector =
      cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.planning_tactical_plugin =
      config_.intersection_transit_plugin_name;
  maneuver_msg.intersection_transit_straight_maneuver.parameters.planning_strategic_plugin =
      config_.strategic_plugin_name;
  maneuver_msg.intersection_transit_straight_maneuver.start_dist = start_dist;
  maneuver_msg.intersection_transit_straight_maneuver.start_speed = start_speed;
  maneuver_msg.intersection_transit_straight_maneuver.start_time = start_time;
  maneuver_msg.intersection_transit_straight_maneuver.end_dist = end_dist;
  maneuver_msg.intersection_transit_straight_maneuver.end_speed = target_speed;
  maneuver_msg.intersection_transit_straight_maneuver.end_time = end_time;
  maneuver_msg.intersection_transit_straight_maneuver.starting_lane_id = std::to_string(starting_lane_id);
  maneuver_msg.intersection_transit_straight_maneuver.ending_lane_id = std::to_string(ending_lane_id);

  // Start time and end time for maneuver are assigned in updateTimeProgress

  ROS_INFO_STREAM("Creating IntersectionTransitManeuver start dist: " << start_dist << " end dist: " << end_dist
                                                                      << " From lanelet: " << starting_lane_id
                                                                      << " to lanelet: " << ending_lane_id
                                                                      << " From start_time: " << start_time
                                                                      << " to end_time: " << end_time);

  return maneuver_msg;
}

double LCIStrategicPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt) const
{
  lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm_->getTrafficRules();
  if (traffic_rules)
  {
    return (*traffic_rules)->speedLimit(llt).speedLimit.value();
  }
  else
  {
    throw std::invalid_argument("Valid traffic rules object could not be built");
  }
}

}  // namespace lci_strategic_plugin