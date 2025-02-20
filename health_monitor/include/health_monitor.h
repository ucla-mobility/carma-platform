#pragma once

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

#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/PluginList.h>
#include <cav_srvs/PluginActivation.h>
#include <cav_msgs/Plugin.h>
#include <cav_msgs/DriverStatus.h>
#include <boost/optional.hpp>
#include "driver_manager.h"

namespace health_monitor
{
    class HealthMonitor
    {
        public:
            
            /*!
             * \brief Default constructor for HealthMonitor
             */
            HealthMonitor();
            /*!
             * \brief Begin normal execution of health monitor node. Will take over control flow of program and exit from here.
             * 
             * \return The exit status of this program
             */
            void run();

            // spin callback function
            bool spin_cb();
            


            //Unit Testing Functions
            void setDriverManager(DriverManager dm);
            void setCarTrue();
            void setTruckTrue();

        private:

            // node handles
            std::shared_ptr<ros::CARMANodeHandle> nh_;
            std::shared_ptr<ros::CARMANodeHandle> pnh_;

            // workers
            DriverManager driver_manager_;

            // topic subscribers
            ros::Subscriber driver_discovery_subscriber_;

            // message/service callbacks
            void driver_discovery_cb(const cav_msgs::DriverStatusConstPtr& msg);

            // initialize method
            void initialize();

            // ROS params
            double spin_rate_, driver_timeout_, startup_duration_;
            std::vector<std::string> required_drivers_;
            std::vector<std::string> lidar_gps_drivers_;
            std::vector<std::string> camera_drivers_; 
 
            bool truck_;
            bool car_;

            // record of startup timestamp
            long start_up_timestamp_;

            // Previously published alert message
            boost::optional<cav_msgs::SystemAlert> prev_alert;
    };
}