/*
 * Copyright (C) 2019-2021 LEIDOS.
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
 * Developed by the UCLA Mobility Lab, 9/9/2022. 
 *
 * Creator: Xu Han
 * Author: Xu Han, Xin Xia, Zonglin Meng, Jiaqi Ma
 */


#include <ros/ros.h>
#include <string>
#include "emergency_pullover_strategic.h"
#include <array>
#include <stdlib.h> 


namespace emergency_pullover_strategic
{
    // EmergencyPulloverStrategicPlugin::EmergencyPulloverStrategicPlugin();
    // -------------------------------- Plug-in constructor --------------------------------
    EmergencyPulloverStrategicPlugin::EmergencyPulloverStrategicPlugin(carma_wm::WorldModelConstPtr wm, 
                                                                       EmergencyPulloverStrategicPluginConfig config,
                                                                       PublishPluginDiscoveryCB plugin_discovery_publisher
                                                                       ):
    plugin_discovery_publisher_(plugin_discovery_publisher),
    wm_(wm),
    config_(config)
    {
        ROS_DEBUG_STREAM("Top of EmergencyPulloverStrategicPlugin actor.");
        std::string hostStaticId = config_.vehicleID; //static ID for this vehicle
        long cur_t = ros::Time::now().toNSec()/1000000; // current time in millisecond
        
        plugin_discovery_msg_.name = "EmergencyPullOverStrategicPlugin";
        plugin_discovery_msg_.version_id = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = true;
        plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
        plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";
        ROS_DEBUG_STREAM("actor complete. hostStaticId = " << hostStaticId);
    }
    
    // -------------------------------- Data extraction --------------------------------
    // Build map projector from projection string (georefernce).
    void EmergencyPulloverStrategicPlugin::georeference_cb(const std_msgs::StringConstPtr& msg) 
    {
        map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(msg->data.c_str()); 
    }

    // Find ecef point based on pose message
    cav_msgs::LocationECEF EmergencyPulloverStrategicPlugin::pose_to_ecef(geometry_msgs::PoseStamped pose_msg)
    {
        // check map projector 
        if (!map_projector_) 
        {
            throw std::invalid_argument("No map projector available for ecef conversion");
        }
        
        // read host location
        cav_msgs::LocationECEF location;
        // note: ecef point read from map projector is in m.
        lanelet::BasicPoint3d ecef_point = map_projector_->projectECEF({pose_msg.pose.position.x, pose_msg.pose.position.y, 0.0}, 1);
        location.ecef_x = ecef_point.x() * 100.0;
        location.ecef_y = ecef_point.y() * 100.0;
        location.ecef_z = ecef_point.z() * 100.0;    
        
        // note: the returned ecef is in cm.
        return location;
    }

    // Function to assign host pose_ecef_point_
    void EmergencyPulloverStrategicPlugin::setHostECEF(cav_msgs::LocationECEF pose_ecef_point)
    {
        // Note, the ecef here is in cm. 
        pose_ecef_point_ = pose_ecef_point;
    }
    
    // Callback to update downtrack and crosstrack based on pose message.
    void EmergencyPulloverStrategicPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        if (!wm_->getRoute())
        return;
    
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());

        // read current location based on pose msg
        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        carma_wm::TrackPos tc = wm_->routeTrackPos(current_loc);

        // update host's DtD and CtD
        current_downtrack_ = tc.downtrack;
        current_crosstrack_ = tc.crosstrack;

        // set host ecef
        // note: the ecef read from "pose_ecef_point" is in cm.
        cav_msgs::LocationECEF pose_ecef_point = pose_to_ecef(pose_msg_);
        setHostECEF(pose_ecef_point);
        
    }

    // callback to update the command speed on x direction, in m/s.
    void EmergencyPulloverStrategicPlugin::cmd_cb(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        cmd_speed_ = msg->twist.linear.x;
    }
   
    // twist command, read linear speed on x direction, in m/s.
    void EmergencyPulloverStrategicPlugin::twist_cb(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
        if (current_speed_ < STOPPED_SPEED)     // note: stopped_speed (threshold of determining stop) is defined in the header file.
        {
            current_speed_ = 0.0;
        }
    }

    // Find the speed limit for the current road (also used as the desired speed).
    double EmergencyPulloverStrategicPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt)
    {
        double target_speed = 0.0;

        lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm_->getTrafficRules();
        if (traffic_rules)
        {
            target_speed =(*traffic_rules)->speedLimit(llt).speedLimit.value();
        }
        else
        {
            throw std::invalid_argument("Valid traffic rules object could not be built");
        }

        ROS_DEBUG_STREAM("target speed (limit) " << target_speed);
        
        return target_speed;
    }

    // Return the ecef point projected to local map point.
    lanelet::BasicPoint2d EmergencyPulloverStrategicPlugin::ecef_to_map_point(cav_msgs::LocationECEF ecef_point)
    {
        if (!map_projector_) 
        {
            throw std::invalid_argument("No map projector available for ecef conversion");
        }

        lanelet::BasicPoint3d map_point = map_projector_->projectECEF(
            {(double)ecef_point.ecef_x/100.0, (double)ecef_point.ecef_y/100.0, (double)ecef_point.ecef_z/100.0 }, -1);
        
        lanelet::BasicPoint2d output {map_point.x(), map_point.y()};
        return output;
    } 

    // note: consider add a function for emergency lane change state 
    
    // -------------------------------- Helper functions --------------------------------

    // callback function to go through external object list and check if emergency vehicle exist
    void EmergencyPulloverStrategicPlugin::external_objects_cb(const cav_msgs::ExternalObjectListConstPtr& msg)
    {
        /**
         * Note: 
         * This function is called in the emergency pullover strategic plugin node, which is running consistently with the node. 
         * So, it is necessary to check for the already detected emergency vehicle. Once detected, host vehicle should
         *  stop searching and send lane change plan right away.
         */
        
        ROS_DEBUG_STREAM("external_objects_callback called");
        
        // loop through objects to check for emergency vehicle 
        for (auto obj : msg->objects) 
        {
            ROS_DEBUG_STREAM("external_objects_callback detect emergency vehicle");
            // found emergency vehicle 
            // if (obj.object_type == cav_msgs::ExternalObject::LARGE_VEHICLE)
            // todo: use "large vehicle" for compilation, need to update perception message and add "emergency vehicle" type.
            if (obj.object_type == cav_msgs::ExternalObject::EMERGENCY_VEHICLE)
            {
                // first time detect 
                if (not_emergency_vehicle_detected_) 
                {
                    // note: inherithed from gnss to map converter, currently the covariance is not used
                    // init a new msg to handle data type
                    geometry_msgs::PoseStamped msg;  // note: obj.pose is of type PoseWithCovariance
                    geometry_msgs::PoseWithCovariance pose_msg = obj.pose;
                    msg.pose = pose_msg.pose;
                    
                    // update emergency vehicle 
                    emergency_vehicle_pose_ = msg;
                    // update emergency vehicle ID
                    emergency_vehicle_id_ = obj.id;
                    
                    // update bool indicator
                    not_emergency_vehicle_detected_ = false;
                    return;
                }
                // repeated detection 
                else
                {
                    // note: inherithed from gnss to map converter, currently the covariance is not used
                    // init a new msg to handle data type
                    geometry_msgs::PoseStamped msg;  // note: obj.pose is of type PoseWithCovariance
                    geometry_msgs::PoseWithCovariance pose_msg = obj.pose;
                    msg.pose = pose_msg.pose;
                    
                    // update emergency vehicle 
                    emergency_vehicle_pose_ = msg;
                    return;
                }
            }
        }    
        // no emergency vehicles found
        ROS_DEBUG_STREAM("external_objects_callback did not detect emergency vehicle.");
        return;    
    }

    // callback function to go through roadway obstacle object list and check if emergency vehicle exist 
    // void EmergencyPulloverStrategicPlugin::roadway_obstacles_cb(const cav_msgs::RoadwayObstacleListConstPtr& msg msg)
    // {
    //     *
    //      * Note: 
    //      * This is an empty function in case roadway obstacles is needed. 
    //      * One of the two will be deleted based on emergency detection implementation.
         
        
    //     ROS_DEBUG_STREAM("roadway_obstacles_callback called");
    //     return; 
    // }
    
    bool EmergencyPulloverStrategicPlugin::isSameLaneWithEmergencyVehicle(geometry_msgs::PoseStamped pose_msg)
    {
        // find ecef location of the emergency vehicle
        emergency_ecef_location_ = pose_to_ecef(pose_msg);
        // find dtd and ctd
        lanelet::BasicPoint2d current_loc(pose_msg.pose.position.x, pose_msg.pose.position.y);
        double emergency_dtd = wm_->routeTrackPos(current_loc).downtrack;
        double emergency_ctd = wm_->routeTrackPos(current_loc).crosstrack;

        ROS_DEBUG_STREAM("Emergency vehicle down track from ecef: " << emergency_dtd << ", cross track from ecef: " << emergency_ctd);

        // check lane position 
        bool is_emergency_same_lane = abs(current_crosstrack_-emergency_ctd) <= config_.maxCrosstrackError;

        return is_emergency_same_lane;
    }

    // check if lane change is feasible 
    bool EmergencyPulloverStrategicPlugin::is_lanechange_possible(lanelet::Id start_lanelet_id, lanelet::Id target_lanelet_id)
    {
        lanelet::ConstLanelet starting_lanelet = wm_->getMap()->laneletLayer.get(start_lanelet_id);
        lanelet::ConstLanelet ending_lanelet = wm_->getMap()->laneletLayer.get(target_lanelet_id);
        lanelet::ConstLanelet current_lanelet = starting_lanelet;
        bool shared_boundary_found = false;
        
        while(!shared_boundary_found)
        {
            //Assumption: Adjacent lanelets share lane boundary
            if(current_lanelet.leftBound() == ending_lanelet.rightBound())
            {   
                ROS_DEBUG_STREAM("Lanelet " << std::to_string(current_lanelet.id()) << " shares left boundary with " << std::to_string(ending_lanelet.id()));
                shared_boundary_found = true;
            }
            else if(current_lanelet.rightBound() == ending_lanelet.leftBound())
            {
                ROS_DEBUG_STREAM("Lanelet " << std::to_string(current_lanelet.id()) << " shares right boundary with " << std::to_string(ending_lanelet.id()));
                shared_boundary_found = true;
            }

            else
            {
                //If there are no following lanelets on route, lanechange should be completing before reaching it
                if(wm_->getMapRoutingGraph()->following(current_lanelet, false).empty())
                {
                    //In this case we have reached a lanelet which does not have a routable lanelet ahead + isn't adjacent to the lanelet where lane change ends
                    return false;
                }

                current_lanelet = wm_->getMapRoutingGraph()->following(current_lanelet, false).front(); 
                if(current_lanelet.id() == starting_lanelet.id())
                {
                    //Looped back to starting lanelet
                    return false;
                }
            }
        }

        return true;
    }

    // Return the lanelet id.
    int EmergencyPulloverStrategicPlugin::findLaneletIndexFromPath(int target_id, lanelet::routing::LaneletPath& path)
    {
        for(size_t i = 0; i < path.size(); ++i)
        {
            if(path[i].id() == target_id)
            {
                return i;
            }
        }
        return -1;
    }

    // compose maneuver message 
    cav_msgs::Maneuver EmergencyPulloverStrategicPlugin::composeManeuverMessage(double current_dist, 
                                                                                                double end_dist, 
                                                                                                double current_speed, 
                                                                                                double target_speed, 
                                                                                                int lane_id, 
                                                                                                ros::Time& current_time)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        // note: need to modify "cav_msgs/msg/Maneuver_parameters --> negotiation_type" to add enum "PULLOVER"
        maneuver_msg.lane_following_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::PULLOVER;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        // use default lane following tactical plug-in
        maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = "InLaneCruisingPlugin";
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = "EmergencyPullOverStrategicPlugin";
        maneuver_msg.lane_following_maneuver.start_dist = current_dist;
        maneuver_msg.lane_following_maneuver.start_speed = current_speed;
        maneuver_msg.lane_following_maneuver.start_time = current_time;
        maneuver_msg.lane_following_maneuver.end_dist = end_dist;
        maneuver_msg.lane_following_maneuver.end_speed = target_speed;
        
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        maneuver_msg.lane_following_maneuver.end_time = current_time + ros::Duration(config_.time_step);
        maneuver_msg.lane_following_maneuver.lane_ids = { std::to_string(lane_id) };

        ROS_DEBUG_STREAM("compose maneuver lane id:"<< lane_id);

        lanelet::ConstLanelet current_lanelet = wm_->getMap()->laneletLayer.get(lane_id);
        if(!wm_->getMapRoutingGraph()->following(current_lanelet, false).empty())
        {

            auto next_lanelet_id = wm_->getMapRoutingGraph()->following(current_lanelet, false).front().id();
            ROS_DEBUG_STREAM("next_lanelet_id:"<< next_lanelet_id);
            maneuver_msg.lane_following_maneuver.lane_ids.push_back(std::to_string(next_lanelet_id));
        }
        else
        {
            ROS_DEBUG_STREAM("No following lanelets");
        }

        current_time = maneuver_msg.lane_following_maneuver.end_time;
        ROS_DEBUG_STREAM(
                    "Creating lane follow start dist:"<<current_dist<<" end dist:"<<end_dist);
        ROS_DEBUG_STREAM(
                    "Duration:"<< maneuver_msg.lane_following_maneuver.end_time.toSec() - maneuver_msg.lane_following_maneuver.start_time.toSec());
        return maneuver_msg;
    }

    // UCLA: compose maneuver message for lane change 
    cav_msgs::Maneuver EmergencyPulloverStrategicPlugin::composeLaneChangeManeuverMessage(double current_dist, 
                                                                                                          double end_dist, 
                                                                                                          double current_speed, 
                                                                                                          double target_speed, 
                                                                                                          int starting_lane_id, 
                                                                                                          int ending_lane_id, 
                                                                                                          ros::Time& current_time)
    {
        cav_msgs::Maneuver maneuver_msg;
        // UCLA: change to lane change maneuvers
        maneuver_msg.type = cav_msgs::Maneuver::LANE_CHANGE;
        // note: need to modify "cav_msgs/msg/Maneuver_parameters --> negotiation_type" to add enum "PULLOVER"
        maneuver_msg.lane_change_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::PULLOVER;
        maneuver_msg.lane_change_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_change_maneuver.parameters.planning_tactical_plugin = "CooperativeLaneChangePlugin";
        maneuver_msg.lane_change_maneuver.parameters.planning_strategic_plugin = "EmergencyPullOverStrategicPlugin";
        maneuver_msg.lane_change_maneuver.start_dist = current_dist;
        maneuver_msg.lane_change_maneuver.start_speed = current_speed;
        maneuver_msg.lane_change_maneuver.start_time = current_time;
        maneuver_msg.lane_change_maneuver.end_dist = end_dist;
        maneuver_msg.lane_change_maneuver.end_speed = target_speed;        
        // Generate a new maneuver ID for the lane change maneuver (not needed for lane following maneuver).
        maneuver_msg.lane_following_maneuver.parameters.maneuver_id = boost::uuids::to_string(boost::uuids::random_generator()());
        
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        double cur_plus_target = current_speed + target_speed;
        if (cur_plus_target < 0.00001) 
        {
            maneuver_msg.lane_change_maneuver.end_time = current_time + ros::Duration(config_.time_step);
        } 
        else 
        {
            // maneuver_msg.lane_change_maneuver.end_time = current_time + ros::Duration((end_dist - current_dist) / (0.5 * cur_plus_target));
            maneuver_msg.lane_change_maneuver.end_time = current_time + ros::Duration(20.0);

        }

        // UCLA: need both start laneID and end laneID  for lane change
        maneuver_msg.lane_change_maneuver.starting_lane_id = { std::to_string(starting_lane_id) };
        maneuver_msg.lane_change_maneuver.ending_lane_id = { std::to_string(ending_lane_id) };

        current_time = maneuver_msg.lane_change_maneuver.end_time;
        ROS_DEBUG_STREAM(
                    "Creating lane change start dist:"<<current_dist<<" end dist:"<<end_dist);
        ROS_DEBUG_STREAM(
                    "Duration:"<< maneuver_msg.lane_change_maneuver.end_time.toSec() - maneuver_msg.lane_change_maneuver.start_time.toSec());
        
        return maneuver_msg;
    }

    // update current status based on maneuver 
    void EmergencyPulloverStrategicPlugin::updateCurrentStatus(cav_msgs::Maneuver maneuver, 
                                                               double& speed, 
                                                               double& current_progress, 
                                                               int& lane_id)
    {
        if(maneuver.type == cav_msgs::Maneuver::LANE_FOLLOWING){
            speed =  maneuver.lane_following_maneuver.end_speed;
            current_progress =  maneuver.lane_following_maneuver.end_dist;
            if (maneuver.lane_following_maneuver.lane_ids.empty()) 
            {
                ROS_DEBUG_STREAM("Lane id of lane following maneuver not set. Using 0");
                lane_id = 0;
            } 
            else 
            {
                lane_id =  stoi(maneuver.lane_following_maneuver.lane_ids[0]);
            }
        }
    }

    // -------------------------------- Generate maneuver plan --------------------------------
    bool EmergencyPulloverStrategicPlugin::plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, 
                                                            cav_srvs::PlanManeuversResponse &resp)
    {
        // use current position to find lanelet ID
        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);

        // *** get the actually closest lanelets that relate to current location (n=10) ***//
        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 10); 

        // --------------- ROS 2 lanelet update --------------------
        // lanelet::ConstLanelet current_lanelet; 
        // // To avoid overlapping lanelets, compare the nearest lanelets with the route
        // for (auto llt: current_lanelets)
        // {
        //     if (wm_->getRoute()->contains(llt.second))
        //     {
        //         current_lanelet = llt.second;
        //         break;
        //     }
        // }

        // raise warn if no path was found
        if(current_lanelets.size() == 0)
        {
            ROS_DEBUG_STREAM("Cannot find any lanelet in map!");
            return true;
        }

        // locate lanelet on shortest path
        auto shortest_path = wm_->getRoute()->shortestPath(); // find path among route

        lanelet::ConstLanelet current_lanelet;
        int last_lanelet_index = -1;
        for (auto llt : current_lanelets)
        {
            if (boost::geometry::within(current_loc, llt.second.polygon2d())) 
            {
                int potential_index = findLaneletIndexFromPath(llt.second.id(), shortest_path); // usage: findLaneletIndexFromPath(target_id, lanelet2_path)
                if (potential_index != -1)
                {
                    last_lanelet_index = potential_index;
                    current_lanelet = shortest_path[last_lanelet_index]; // find lanelet2 from map that corresponse to the path
                    break;
                }
            }
        }


        // read status data
        double current_progress = wm_->routeTrackPos(current_loc).downtrack;
        double speed_progress = current_speed_;
        ros::Time time_progress = ros::Time::now();
        
        // find speed limit 
        double speed_limit = findSpeedLimit(current_lanelet);   //get Speed Limit
        
        // pick smaller length, accommodate when host is close to the route end
        double route_length =  wm_->getRouteEndTrackPos().downtrack; 
       
        // Update current status based on prior plan
        if(req.prior_plan.maneuvers.size()!= 0)
        {
            ROS_DEBUG_STREAM("Provided with initial plan...");
            time_progress = req.prior_plan.planning_completion_time;
            int end_lanelet = 0;
            updateCurrentStatus(req.prior_plan.maneuvers.back(), speed_progress, current_progress, end_lanelet);
            last_lanelet_index = findLaneletIndexFromPath(end_lanelet, shortest_path);
        }
        
        ROS_DEBUG_STREAM("Starting Loop");
        // Note: Use current_lanelet list (which was determined based on vehicle pose) to find current lanelet ID. 
        // lanelet::Id current_lanelet_id = current_lanelet.id();
        long current_lanelet_id = current_lanelets[0].second.id();
        ROS_DEBUG_STREAM("current_lanelet_id: " << current_lanelet_id);

        // check emergency vehicle position and send maneuver plan
        // no emergency vehicle detected, maintain current lane and speed
        ROS_DEBUG_STREAM("not_emergency_vehicle_detected_: " << not_emergency_vehicle_detected_);
        if (not_emergency_vehicle_detected_)
        // if (false)
        {   
            double target_speed = speed_limit;
            double total_maneuver_length = current_progress + config_.time_step * target_speed;
            total_maneuver_length = std::min(total_maneuver_length, route_length);
            ROS_DEBUG_STREAM("total_maneuver_length: " << total_maneuver_length << " route_length: " << route_length);


            ROS_DEBUG_STREAM("Planning Lane Following Maneuver, no emergency vehicle detected! ");
            
            while (current_progress < total_maneuver_length)
            {   
                ROS_DEBUG_STREAM("Lane Following Maneuver! (No emergency vehicle detected)");
                ROS_DEBUG_STREAM("current_progress: "<< current_progress);
                ROS_DEBUG_STREAM("speed_progress: " << speed_progress);
                ROS_DEBUG_STREAM("target_speed: " << target_speed);
                ROS_DEBUG_STREAM("time_progress: " << ros::Time(time_progress).toSec());
                double end_dist = total_maneuver_length;
                ROS_DEBUG_STREAM("end_dist: " << end_dist);
                double dist_diff = end_dist - current_progress;
                ROS_DEBUG_STREAM("dist_diff: " << dist_diff);
                if (end_dist < current_progress)
                {
                    break;
                }

                resp.new_plan.maneuvers.push_back(composeManeuverMessage(current_progress, end_dist,  
                                        speed_progress, target_speed, current_lanelet_id, time_progress));
                                    

                current_progress += dist_diff;
                time_progress = resp.new_plan.maneuvers.back().lane_following_maneuver.end_time;
                speed_progress = target_speed;
                if(current_progress >= total_maneuver_length || last_lanelet_index == static_cast<int>(shortest_path.size()) - 1)
                {
                    break;
                }

                // update lanelet index
                ++last_lanelet_index;
            }
        }
        // emergency vehicle detected, check relative position
        else
        {   
            // check emergency vehicle position
            // (select right lanelet is inherited from "/basic_autonomy_ros_2/basic_autonomy.cpp")

            // same lane with host, prepare to pull-over
            if (true) 
            {
                // right lane exist 
                if(true)
                {
                    // init variables
                    double target_speed;
                    double total_maneuver_length;
                    lanelet::ConstLanelet target_lanelet;
                    lanelet::Id target_lanelet_id;
                    // mark vehicle as pulled-over (default as false)
                    is_host_pulled_over_ = true;
                    
                    // if right lane available
                    if (wm_->getMapRoutingGraph()->right(current_lanelet))
                    {
                        // locate target lanelet
                        target_lanelet = wm_->getMapRoutingGraph()->right(current_lanelet).get();
                        target_lanelet_id = target_lanelet.id();

                        // emergency vehicle detected, start lane change with reduced speed
                        target_speed = speed_limit*config_.lane_change_speed_adjustment;
                        total_maneuver_length = current_progress + config_.time_step * target_speed;
                        ROS_DEBUG_STREAM("Planning right Lane Change Maneuver, same-lane emergency vehicle detected! ");
                        total_maneuver_length = std::min(total_maneuver_length, route_length);
                        ROS_DEBUG_STREAM("total_maneuver_length: " << total_maneuver_length << " route_length: " << route_length);

                    }
                    
                    // if adjacent right lane available
                    else
                    {
                        // locate target lanelet 
                        target_lanelet = wm_->getMapRoutingGraph()->adjacentRight(current_lanelet).get();
                        target_lanelet_id = target_lanelet.id();

                        // emergency vehicle detected, start lane change with reduced speed
                        target_speed = speed_limit*config_.lane_change_speed_adjustment;
                        total_maneuver_length = current_progress + config_.time_step * target_speed;
                        ROS_DEBUG_STREAM(
                                    "Planning adjacent right Lane Change Maneuver, same-lane emergency vehicle detected!");
                        total_maneuver_length = std::min(total_maneuver_length, route_length);
                        ROS_DEBUG_STREAM("total_maneuver_length: " << total_maneuver_length << " route_length: " << route_length);

                    }

                    // send out lane change plan
                    while (current_progress < total_maneuver_length)
                    {   
                        ROS_DEBUG_STREAM("Lane Change Maneuver for emergency pull-over! ");
                        ROS_DEBUG_STREAM("current_progress: "<< current_progress);
                        ROS_DEBUG_STREAM("speed_progress: " << speed_progress);
                        ROS_DEBUG_STREAM("target_speed: " << target_speed);
                        ROS_DEBUG_STREAM("time_progress: " << ros::Time(time_progress).toSec());

                        // set to next lane destination
                        double end_dist = total_maneuver_length;
                        ROS_DEBUG_STREAM("end_dist: " << end_dist);
                        // consider calculate dtd_diff and ctd_diff
                        double dist_diff = end_dist - current_progress;
                        ROS_DEBUG_STREAM("dist_diff: " << dist_diff);
                        // define desired downtrack distance needed to finish lane change (in terms of number of maneuvers length)
                        double lc_end_dist = config_.lane_change_desired_steps*total_maneuver_length;
                        ROS_DEBUG_STREAM("lc_end_dist before buffer: " << lc_end_dist);
                        lc_end_dist = std::max(lc_end_dist, current_progress + config_.maxLaneChangeDist);
                        ROS_DEBUG_STREAM("lc_end_dist after buffer: " << lc_end_dist);
                        
                        // Check if lane change possible
                        bool lanechangePossible = true;
                        ROS_DEBUG_STREAM("lanechangePossible: " << lanechangePossible);
                        
                        // send out plan
                        if (lanechangePossible)
                        {
                            ROS_DEBUG_STREAM("Lane change possible, planning it.. " );
                            resp.new_plan.maneuvers.push_back(composeLaneChangeManeuverMessage(current_downtrack_, lc_end_dist,  
                                                speed_progress, target_speed, current_lanelet_id, target_lanelet_id , time_progress));
                            
                        }
                        else
                        {
                            ROS_DEBUG_STREAM("Lane change impossible, planning lane following with reduced speed ... " );
                            resp.new_plan.maneuvers.push_back(composeManeuverMessage(current_downtrack_, end_dist,  
                                                speed_progress, target_speed, current_lanelet_id, time_progress));
                        }

                        // Update
                        current_progress += dist_diff;
                        // read lane change maneuver end time as time progress
                        time_progress = resp.new_plan.maneuvers.back().lane_change_maneuver.end_time;
                        speed_progress = target_speed;
                        if(current_progress >= total_maneuver_length || last_lanelet_index == static_cast<int>(shortest_path.size()) - 1)
                        {
                            break;
                        }

                        // update lanelet index
                        ++last_lanelet_index;

                    }
                
                }            

            }        
        }
        return true;
    }

    // -------------------------------- Spin Emergency Pullover plugin --------------------------------
    bool EmergencyPulloverStrategicPlugin::onSpin() 
    {  
        plugin_discovery_publisher_(plugin_discovery_msg_);      
        ROS_DEBUG_STREAM("Emergency pull-over plug-in is on Spin... ");
        if (not_emergency_vehicle_detected_)
        {
            ROS_DEBUG_STREAM("No emergency vehicle detected... ");
        }
        else
        {
            if(is_host_pulled_over_)
            {
                ROS_DEBUG_STREAM("Emergency vehicle detected, host started lane change... ");
            }
            else
            {
                ROS_DEBUG_STREAM("Emergency vehicle detected, host searching for lane change opportunity... ");
            }
        }

        return true;
    }
    
} //emergency_pullover_strategic