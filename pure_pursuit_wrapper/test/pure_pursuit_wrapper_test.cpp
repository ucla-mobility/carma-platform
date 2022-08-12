/*
 * Copyright (C) 2018-2022 LEIDOS.
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

#include "pure_pursuit_wrapper/pure_pursuit_wrapper.hpp"

#include <gtest/gtest.h>
#include <iostream>
#include <boost/optional/optional.hpp>

/*
TEST(pure_pursuit_wrapper, trajectoryPlanHandler)
{
  cav_msgs::TrajectoryPlan plan;
  plan.initial_longitudinal_velocity = 8.5;

  boost::optional<autoware_msgs::Lane> wp_msg;
  boost::optional<carma_planning_msgs::msg::Plugin> plugin_msg;
  pure_pursuit_wrapper::PurePursuitWrapperConfig config;
  pure_pursuit_wrapper::PurePursuitWrapper ppw(config, [&wp_msg](auto msg) { wp_msg = msg; },
                                               [&plugin_msg](auto msg) { plugin_msg = msg; });

  carma_planning_msgs::msg::TrajectoryPlanPoint tpp, tpp2, tpp3;
  tpp.x = 10;
  tpp.y = 10;
  tpp.target_time = rclcpp::Time(0.1);  // 8.5m/s

  tpp2.x = 12;
  tpp2.y = 12;
  tpp2.target_time = rclcpp::Time(0.2);  // 48.068542495 m/s

  tpp3.x = 14;
  tpp3.y = 14;
  tpp3.target_time = rclcpp::Time(0.3);  // 8.5m/s

  plan.trajectory_points = { tpp, tpp2, tpp3 };

  carma_planning_msgs::msg::TrajectoryPlan::UniquePtr plan_ptr(new cav_msgs::TrajectoryPlan(plan));

  ASSERT_FALSE(!!wp_msg);

  ppw.trajectoryPlanHandler(plan_ptr);

  ASSERT_TRUE(!!wp_msg);

  autoware_msgs::Lane lane = wp_msg.get();

  ASSERT_EQ(3, lane.waypoints.size());
  ASSERT_NEAR(8.5, lane.waypoints[0].twist.twist.linear.x, 0.0000001);
  ASSERT_NEAR(10.0, lane.waypoints[0].pose.pose.position.x, 0.0000001);
  ASSERT_NEAR(10.0, lane.waypoints[0].pose.pose.position.y, 0.0000001);

  ASSERT_NEAR(48.068542495, lane.waypoints[1].twist.twist.linear.x, 0.0000001);
  ASSERT_NEAR(12.0, lane.waypoints[1].pose.pose.position.x, 0.0000001);
  ASSERT_NEAR(12.0, lane.waypoints[1].pose.pose.position.y, 0.0000001);

  ASSERT_NEAR(8.5, lane.waypoints[2].twist.twist.linear.x, 0.0000001);
  ASSERT_NEAR(14.0, lane.waypoints[2].pose.pose.position.x, 0.0000001);
  ASSERT_NEAR(14.0, lane.waypoints[2].pose.pose.position.y, 0.0000001);
}
*/

