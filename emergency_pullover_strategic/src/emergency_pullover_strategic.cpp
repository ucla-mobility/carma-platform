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
                                                                       PublishPluginDiscoveryCB plugin_discovery_publisher,
                                                                       StopVehicleCB stop_vehicle_publisher
                                                                       ):
    plugin_discovery_publisher_(plugin_discovery_publisher),
    stop_vehicle_publisher_(stop_vehicle_publisher),
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

    // twist command, read linear speed on x direction, in m/s.
    void EmergencyPulloverStrategicPlugin::route_cb(const cav_msgs::RouteConstPtr& msg)
    {
        route_msg_ = cav_msgs::Route(*msg.get());
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
    
    // -------------------------------- Helper functions --------------------------------

    // callback function to go through external object list and check if emergency vehicle exist
    void EmergencyPulloverStrategicPlugin::external_objects_cb(const cav_msgs::ExternalObjectListConstPtr& msg)
    {
        /**
         * Note: 
         * This function is called in the emergency pullover strategic plugin node, which is running consistently with the node. 
         * So, it is necessary to check for the already detected emergency vehicle. Once detected, host vehicle should
         *  stop searching and send lane change plan right away. It is assumed only one emergency vehicle will appear.
         */
        
        // loop through objects to check for emergency vehicle 
        for (auto obj : msg->objects) 
        {
            // Note: add "emergency vehicle" type to cav_msgs/external_object
            if (obj.object_type == cav_msgs::ExternalObject::EMERGENCY_VEHICLE)
            {
                // first time detect 
                if (not_emergency_vehicle_detected_) 
                {
                    // update bool indicator if 1st detected
                    not_emergency_vehicle_detected_ = false;
                }

                // init a new msg to handle data type
                geometry_msgs::PoseStamped msg;
                geometry_msgs::PoseWithCovariance pose_msg = obj.pose;
                msg.pose = pose_msg.pose;
                
                // update emergency vehicle 
                emergency_vehicle_pose_ = msg;
                // update emergency vehicle ID
                emergency_vehicle_id_ = obj.id;
                
                return;
            }
        }    
        // no emergency vehicles found
        // ROS_DEBUG_STREAM("external_objects_callback did not detect emergency vehicle.");
        return;    
    }

    void EmergencyPulloverStrategicPlugin::lane_change_finish_manual_cb(const std_msgs::BoolConstPtr& msg)
    {
        lane_change_finished_manual=true;
        ROS_DEBUG_STREAM("lane_change_finished_manual received");
    }
    
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
        // current position and lanelets
        lanelet::BasicPoint2d current_loc(pose_msg_.pose.position.x, pose_msg_.pose.position.y);
        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 10); 

        // raise warn if no path was found
        if(current_lanelets.size() == 0)
        {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
            return true;
        }

        // locate lanelet on shortest path
        auto shortest_path = wm_->getRoute()->shortestPath(); // find path among route
        lanelet::ConstLanelet target_lanelet; 

        // lanelet::ConstLanelet current_lanelet;
        int last_lanelet_index = -1;
        for (auto llt : current_lanelets)
        {
            if (boost::geometry::within(current_loc, llt.second.polygon2d())) 
            {
                int potential_index = findLaneletIndexFromPath(llt.second.id(), shortest_path); // usage: findLaneletIndexFromPath(target_id, lanelet2_path)
                if (potential_index != -1)
                {
                    last_lanelet_index = potential_index;
                    break;
                }
            }
        }

        // auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 1); 
        if (current_lanelets.empty())
        {
            ROS_DEBUG_STREAM("The current laneletssssss are empty. So no lane change!");
        } 
        lanelet::ConstLanelet current_lanelet = current_lanelets[0].second;


        // read status data
        double current_progress = wm_->routeTrackPos(current_loc).downtrack;
        ROS_DEBUG_STREAM("current downtrack: " << current_progress);
        double speed_progress = current_speed_;
        ros::Time time_progress = ros::Time::now();
        
        // generate target data
        double target_speed;  
        target_speed = findSpeedLimit(current_lanelet);   //get Speed Limit
        double total_maneuver_length = current_progress + config_.time_step * target_speed;
        double route_length =  wm_->getRouteEndTrackPos().downtrack; 
        total_maneuver_length = std::min(total_maneuver_length, route_length);
       
        // Update current status based on prior plan
        if(req.prior_plan.maneuvers.size()!= 0)
        {
            ROS_DEBUG_STREAM("Provided with initial plan...");
            time_progress = req.prior_plan.planning_completion_time;
            ROS_DEBUG_STREAM("Prior plan time progress...xxx"<<time_progress);
            int end_lanelet = 0;
            updateCurrentStatus(req.prior_plan.maneuvers.back(), speed_progress, current_progress, end_lanelet);
            last_lanelet_index = findLaneletIndexFromPath(end_lanelet, shortest_path);
        }
        
        ROS_DEBUG_STREAM("Starting Loop");
        ROS_DEBUG_STREAM("total_maneuver_length: " << total_maneuver_length << " route_length: " << route_length);
        // lane change status
        bool emergency_vehicle_detected = !not_emergency_vehicle_detected_;
        ROS_DEBUG_STREAM("in mvr callback emergency vehicle detection result: " << emergency_vehicle_detected);

        // Note: Use current_lanelet list (which was determined based on vehicle pose) to find current lanelet ID. 
        long current_lanelet_id = current_lanelets[0].second.id();
        ROS_DEBUG_STREAM("current_lanelet_id: " << current_lanelet_id);
        
        // emergency vehicle detected: lane change maneuver
        if (emergency_vehicle_detected)
        {   
            ROS_DEBUG_STREAM("Start to handle emergence vehicle detected case");
            // init right lane indicator 
            bool right_lane_exist;
            bool isLaneChangeFinished;
            lanelet::BasicPoint2d target_lane_center_loc;
            
            // target_lanelet = wm_->getMapRoutingGraph()->right(current_lanelet).get();
            // ROS_DEBUG_STREAM("target_lanelet.id(): " << target_lanelet.id());
            ROS_DEBUG_STREAM("Planning lane change maneuver with right lane!");
            
            // send out lane change plan
            while (current_progress < total_maneuver_length)
            {   
                ROS_DEBUG_STREAM("Lane Change Maneuver ! ");
                ROS_DEBUG_STREAM("current_progress: "<< current_progress);
                ROS_DEBUG_STREAM("speed_progress: " << speed_progress);
                ROS_DEBUG_STREAM("target_speed: " << target_speed);
                ROS_DEBUG_STREAM("time_progress: " << time_progress.toSec());

                // lane change maneuver plan variables 
                double end_dist = total_maneuver_length;
                ROS_DEBUG_STREAM("end_dist: " << end_dist);
                double dist_diff = end_dist - current_progress;
                ROS_DEBUG_STREAM("dist_diff: " << dist_diff);
                double lc_end_dist = total_maneuver_length;
                ROS_DEBUG_STREAM("lc_end_dist: " << lc_end_dist);

                // find target lanelet for lane change
                lanelet::ConstLanelet starting_lanelet = wm_->getMap()->laneletLayer.get(current_lanelet_id);
                lanelet::ConstLanelet target_lanelet;
                bool lanechangePossible;
                int target_lanelet_id;
                int ite = 0;
                
                for(auto id : route_msg_.route_path_lanelet_ids)
                {
                    // find lanelet relation
                    auto llt = wm_->getMap()->laneletLayer.get(id);
                    auto relation = wm_->getMapRoutingGraph()->routingRelation(starting_lanelet, llt);
                    ite++;

                    // if right lane exist
                    if (relation == lanelet::routing::RelationType::Right)
                    {
                        // select target lanelet
                        target_lanelet = llt;
                        target_lanelet_id = id;
                        ROS_DEBUG_STREAM("Lane change possible, right side target_lanelet_id: " << id);
                        lanechangePossible = true;

                        // monitor lane change status
                        target_lane_center_loc = target_lanelet.centerline2d().back();
                        double target_downtrack = wm_->routeTrackPos(target_lane_center_loc).downtrack;

                        // look for connecting lanelet if the current right lanelet is about to end
                        while(target_downtrack<lc_end_dist)
                        {
                            if (!wm_->getMapRoutingGraph()->following(current_lanelet, false).empty())
                            {
                                target_lanelet = wm_->getMapRoutingGraph()->following(current_lanelet, false).front();
                                target_lane_center_loc = target_lanelet.centerline2d().back();
                                target_downtrack=wm_->routeTrackPos(target_lane_center_loc).downtrack;
                                target_lanelet_id=target_lanelet.id();
                            }else
                            {
                                ROS_DEBUG_STREAM(" Can not find target lanelet with enough downtrack! ");
                                lanechangePossible=false;
                                break;
                            }
                        }                   
                            
                        // monitor 
                        double target_crosstrack = wm_->routeTrackPos(target_lane_center_loc).crosstrack;
                        ROS_DEBUG_STREAM("Find target lanelet, start to monitor lane change status... ");
                        ROS_DEBUG_STREAM("target_crosstrack: " << target_crosstrack);
                        ROS_DEBUG_STREAM("current_crosstrack_: " << current_crosstrack_);
                        double crosstrackDiff = current_crosstrack_ - target_crosstrack; 
                        lane_change_finished_ = abs(crosstrackDiff) <= config_.lane_width*config_.em_lane_ctd_check_ratio; 
                        ROS_DEBUG_STREAM("crosstrackDiff: " << crosstrackDiff);
                        ROS_DEBUG_STREAM("is LaneChange Finished: " << lane_change_finished_); //TODO: use this variable to determine lane-change status 

                        break;
                    }

                    // if loop ends with no right lane, mark lane change impossible and continue lane following
                    if (ite == route_msg_.route_path_lanelet_ids.size())
                    {
                        ROS_DEBUG_STREAM("No right lane found, mark lane change impossible...");
                        lanechangePossible = false;    
                    }
                
                }

                // send plan 
                if (lanechangePossible)
                {
                    ROS_DEBUG_STREAM("Lane change possible, planning it.. " );
                    cav_msgs::Maneuver maneuver_msg_=composeLaneChangeManeuverMessage(current_progress, lc_end_dist,  
                                        speed_progress, target_speed, current_lanelet_id, target_lanelet_id , time_progress);
                    resp.new_plan.maneuvers.push_back(maneuver_msg_);
                    resp.new_plan.planning_completion_time=maneuver_msg_.lane_change_maneuver.end_time;
                    ROS_DEBUG_STREAM("resp.new_plan.planning_completion_time = "<<resp.new_plan.planning_completion_time);                        
                }
                else
                {
                    // use reduced target speed after lane change
                    target_speed = target_speed*config_.lane_change_speed_adjustment;   //get Speed Limit
                    total_maneuver_length = current_progress + config_.time_step * target_speed;
                    total_maneuver_length = std::min(total_maneuver_length, route_length);
                    double end_dist = total_maneuver_length;
                    // mark lane change finish position
                    if (lane_change_finish_dtd_ == 0.0) {lane_change_finish_dtd_ = total_maneuver_length;}

                    // log maneuver data
                    ROS_DEBUG_STREAM("Same Lane Maneuver after lane change (right lane not available)! ");
                    ROS_DEBUG_STREAM("current_progress: "<< current_progress);
                    ROS_DEBUG_STREAM("speed_progress: " << speed_progress);
                    ROS_DEBUG_STREAM("target_speed (reduced): " << target_speed);
                    ROS_DEBUG_STREAM("time_progress: " << time_progress.toSec());
                    ROS_DEBUG_STREAM("end_dist: " << end_dist);

                    // calculate distance increment
                    double dist_diff = end_dist - current_progress;
                    ROS_DEBUG_STREAM("dist_diff (after lane change): " << dist_diff);
                    ROS_DEBUG_STREAM("current_lanelet_id: (after lane change) " << current_lanelet_id);

                    ROS_DEBUG_STREAM("Lane change impossible, planning lane following instead ... " );
                    cav_msgs::Maneuver maneuver_msg_=composeManeuverMessage(current_progress, end_dist,  
                                        speed_progress, target_speed, current_lanelet_id, time_progress);
                    resp.new_plan.maneuvers.push_back(maneuver_msg_);
                    resp.new_plan.planning_completion_time=maneuver_msg_.lane_following_maneuver.end_time;
                    ROS_DEBUG_STREAM("resp.new_plan.planning_completion_time = "<<resp.new_plan.planning_completion_time);

                    // send stop notice once lane following dtd in emergency lane pass threshold (vehicle length)
                    if (current_downtrack_ - lane_change_finish_dtd_ >= config_.vehicle_length)
                    {
                        // publish stop notice 
                        std_msgs::Bool stop_v_msg;
                        stop_v_msg.data = true;
                        stop_vehicle_publisher_(stop_v_msg);
                        ROS_DEBUG_STREAM("Publish notice to stop vehicle... ");
                    }
                } // send plan (lane change vs in lane cruising)

                // update lane change variables 
                current_progress += dist_diff;
                time_progress = resp.new_plan.maneuvers.back().lane_change_maneuver.end_time;
                speed_progress = target_speed;
                if(current_progress >= total_maneuver_length)
                {
                    break;
                }

                ++last_lanelet_index;
            }// while loop send out lane change plan

            // Note: lane change finished was determined by checking right lane existance. 
            //TODO: finish this condition to better ditinguish "lane change over" vs "no current right lane".
        
        } // emergency vehicle detected
        
        // emergency vehicle not detected: same-lane maneuver  
        else 
        {
            ROS_DEBUG_STREAM("Planning Same Lane Maneuver! ");
            while (current_progress < total_maneuver_length)
            {   
                ROS_DEBUG_STREAM("Same Lane Maneuver before emergency vehicle detection! ");
                ROS_DEBUG_STREAM("current_progress: "<< current_progress);
                ROS_DEBUG_STREAM("speed_progress: " << speed_progress);
                ROS_DEBUG_STREAM("target_speed: " << target_speed);
                ROS_DEBUG_STREAM("time_progress: " << time_progress.toSec());
                double end_dist = total_maneuver_length;
                ROS_DEBUG_STREAM("end_dist: " << end_dist);
                double dist_diff = end_dist - current_progress;
                ROS_DEBUG_STREAM("dist_diff: " << dist_diff);
                if (end_dist < current_progress)
                {
                    break;
                }

                cav_msgs::Maneuver maneuver_msg_=composeManeuverMessage(current_progress, end_dist,  
                                            speed_progress, target_speed, current_lanelet_id, time_progress);
                resp.new_plan.maneuvers.push_back(maneuver_msg_);
                resp.new_plan.planning_completion_time=maneuver_msg_.lane_following_maneuver.end_time;
                ROS_DEBUG_STREAM("resp.new_plan.planning_completion_time = "<<resp.new_plan.planning_completion_time);
                                    
                current_progress += dist_diff;
                time_progress = resp.new_plan.maneuvers.back().lane_following_maneuver.end_time;
                speed_progress = target_speed;
                if(current_progress >= total_maneuver_length)
                {
                    break;
                }
                ++last_lanelet_index;
            }
        } //emergency vehicle not detected

        if(resp.new_plan.maneuvers.size() == 0)
        {
            ROS_WARN_STREAM("Cannot plan maneuver because no route is found");
        }  
        
        return true;
    } //plan_maneuver_cb;

    // -------------------------------- Spin Emergency Pullover plugin --------------------------------
    bool EmergencyPulloverStrategicPlugin::onSpin() 
    {  
        plugin_discovery_publisher_(plugin_discovery_msg_);      
        // ROS_DEBUG_STREAM("Emergency pull-over plug-in is on Spin... ");
        if (not_emergency_vehicle_detected_)
        {
            // ROS_DEBUG_STREAM("No emergency vehicle detected... ");
        }
        else
        {
            if(is_pull_over_initiated)
            {
                // ROS_DEBUG_STREAM("Emergency vehicle detected, host started lane change... ");
            }
            else
            {
                // ROS_DEBUG_STREAM("Emergency vehicle detected, host searching for lane change opportunity... ");
            }
        }

        return true;
    }
    
} //emergency_pullover_strategic