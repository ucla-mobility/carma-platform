#pragma once

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

/*
 * Developed by the UCLA Mobility Lab, 10/20/2021. 
 *
 * Creator: Xu Han
 * Author: Xu Han, Xin Xia, Jiaqi Ma
 */

#include <cav_msgs/Plugin.h>
#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/PlanTrajectory.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityOperation.h>

#include <cav_msgs/ExternalObjectList.h>
#include <cav_msgs/ExternalObject.h>
#include <cav_msgs/RoadwayObstacleList.h>
#include <cav_msgs/ExternalObject.h>

#include <std_msgs/Bool.h>
#include <cav_msgs/BSM.h>
#include <carma_wm/WMListener.h>
#include <functional>

#include "emergency_pullover_strategic.h"
#include "emergency_pullover_strategic_config.h"

namespace emergency_pullover_strategic
{
/**
 * \brief ROS node for the YieldPlugin
 */ 
class EmergencyPulloverStrategicPluginNode
{
    public:
        /**
         * \brief Entrypoint for this node
         */ 
        void run()
        {
            ros::CARMANodeHandle nh;
            ros::CARMANodeHandle pnh("~");

            carma_wm::WMListener wml;
            auto wm_ = wml.getWorldModel();

            ros::Publisher discovery_pub = nh.advertise<cav_msgs::Plugin>("plugin_discovery", 1);
            ros::Publisher stop_pub = nh.advertise<std_msgs::Bool>("emergency_stop", 10);

            // load config file
            EmergencyPulloverStrategicPluginConfig config;

            // get parameters
            pnh.param<bool>("detecting_emergency_in_front", config.detecting_emergency_in_front, config.detecting_emergency_in_front);
            
            pnh.param<int>("lane_change_desired_steps", config.lane_change_desired_steps, config.lane_change_desired_steps);
            
            pnh.param<double>("time_step", config.time_step, config.time_step);
            pnh.param<double>("lane_change_speed_adjustment", config.lane_change_speed_adjustment, config.lane_change_speed_adjustment);
            pnh.param<double>("maxLaneChangeDist", config.maxLaneChangeDist, config.maxLaneChangeDist);
            pnh.param<double>("stopping_deceleration", config.stopping_deceleration, config.stopping_deceleration);
            pnh.param<double>("lane_change_recheck_time", config.lane_change_recheck_time, config.lane_change_recheck_time);
            pnh.param<double>("lane_width", config.lane_width, config.lane_width);
            pnh.param<double>("vehicle_length", config.vehicle_length, config.vehicle_length);
            pnh.param<double>("em_lane_ctd_check_ratio", config.em_lane_ctd_check_ratio, config.em_lane_ctd_check_ratio);
            pnh.param<double>("em_lane_maintain_ratio", config.em_lane_maintain_ratio, config.em_lane_maintain_ratio);
            pnh.param<double>("reduced_lane_follow_speed", config.reduced_lane_follow_speed, config.reduced_lane_follow_speed);
            
            pnh.getParam("/vehicle_id", config.vehicleID);
            
            ROS_INFO_STREAM("EmergencyPulloverStrategicPluginConfig Params" << config);

            // init worker
            EmergencyPulloverStrategicPlugin worker(wm_, config, 
                                                    [&discovery_pub](auto msg) { discovery_pub.publish(msg); },
                                                    [&stop_pub](auto msg) { stop_pub.publish(msg); });
          
            ros::ServiceServer maneuver_srv_ = nh.advertiseService("plugins/EmergencyPullOverStrategicPlugin/plan_maneuvers",
                                                    &EmergencyPulloverStrategicPlugin::plan_maneuver_cb, &worker);
            
            // external object subs 
            // (note: the topic name was following "roadway object" and "motion computation" node. May need to change here to "external object list")
            ros::Subscriber external_obj_sub = nh.subscribe("external_objects", 1, &EmergencyPulloverStrategicPlugin::external_objects_cb,  &worker);
            ros::Subscriber lane_change_finished_sub = nh.subscribe("lane_change_finish_manual", 1, &EmergencyPulloverStrategicPlugin::lane_change_finish_manual_cb,  &worker);
            // ros::Subscriber roadway_obstacles_sub = nh.subscribe("roadway_obstacles", 1, &EmergencyPulloverStrategicPlugin::roadway_obstacles_cb,  &worker);
            
            // vehicle state subs
            ros::Subscriber current_pose_sub = nh.subscribe("current_pose", 1, &EmergencyPulloverStrategicPlugin::pose_cb,  &worker);
            ros::Subscriber current_twist_sub = nh.subscribe("current_velocity", 1, &EmergencyPulloverStrategicPlugin::twist_cb,  &worker);
            ros::Subscriber cmd_sub = nh.subscribe("twist_raw", 1, &EmergencyPulloverStrategicPlugin::cmd_cb,  &worker);
            ros::Subscriber georeference_sub = nh.subscribe("georeference", 1, &EmergencyPulloverStrategicPlugin::georeference_cb, &worker);    
            ros::Subscriber route_sub = nh.subscribe("route", 1, &EmergencyPulloverStrategicPlugin::route_cb, &worker);
            
            ros::Timer discovery_pub_timer_ = nh.createTimer(
                    ros::Duration(ros::Rate(10.0)),
                    [&worker](const auto&) { worker.onSpin(); });

            ros::CARMANodeHandle::spin();
        }
};

}  // emergency_pullover_strategic
