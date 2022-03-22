#include "message_to_external_object_convertor.h"
#include <carma_v2x_msgs/mobility_path.hpp>
#include <carma_perception_msgs/external_object.hpp>

namespace object
{

    class MobilityPathToExternalObject : public MessageToExternalObjectConvertor<carma_v2x_msgs::msg::MobilityPath>
    {

        void convert(const carma_v2x_msgs::msg::MobilityPath &in_msg, carma_perception_msgs::msg::ExternalObject &out_msg)
        {

            out_msg.size.x = 2.5; // TODO identify better approach for object side in mobility path
            out_msg.size.y = 2.25;
            out_msg.size.z = 2.0;

            // get reference origin in ECEF (convert from cm to m)
            double ecef_x = (double)in_msg.trajectory.location.ecef_x / 100.0;
            double ecef_y = (double)in_msg.trajectory.location.ecef_y / 100.0;
            double ecef_z = (double)in_msg.trajectory.location.ecef_z / 100.0;

            // Convert general information
            out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::ID_PRESENCE_VECTOR;
            out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::POSE_PRESENCE_VECTOR;
            out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::VELOCITY_PRESENCE_VECTOR;
            out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::OBJECT_TYPE_PRESENCE_VECTOR;
            out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::BSM_ID_PRESENCE_VECTOR;
            out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::DYNAMIC_OBJ_PRESENCE;
            out_msg.presence_vector |= carma_perception_msgs::msg::ExternalObject::PREDICTION_PRESENCE_VECTOR;
            out_msg.object_type = carma_perception_msgs::msg::ExternalObject::SMALL_VEHICLE;
            std::hash<std::string> hasher;
            auto hashed = hasher(in_msg.m_header.sender_id); // TODO hasher returns size_t, message accept uint32_t which we might lose info
            out_msg.id = (uint32_t)hashed;

            // convert hex std::string to uint8_t array
            for (size_t i = 0; i < in_msg.m_header.sender_bsm_id.size(); i += 2)
            {
                int num = 0;
                sscanf(in_msg.m_header.sender_bsm_id.substr(i, i + 2).c_str(), "%x", &num);
                out_msg.bsm_id.push_back((uint8_t)num);
            }

            // first point's timestamp
            out_msg.header.stamp = rclcpp::Time((uint64_t)in_msg.m_header.timestamp * 1e6); // ms to nanoseconds

            // If it is a static object, we finished processing
            if (in_msg.trajectory.offsets.size() < 2)
            {
                out_msg.dynamic_obj = false;
                return;
            }
            out_msg.dynamic_obj = true;

            // get planned trajectory points
            carma_perception_msgs::msg::PredictedState prev_state;
            tf2::Vector3 prev_pt_ecef{ecef_x, ecef_y, ecef_z};

            auto prev_pt_map = transform_to_map_frame(prev_pt_ecef);
            double prev_yaw = 0.0;

            double message_offset_x = 0.0; // units cm
            double message_offset_y = 0.0;
            double message_offset_z = 0.0;

            for (size_t i = 0; i < in_msg.trajectory.offsets.size(); i++)
            {
                auto curr_pt_msg = in_msg.trajectory.offsets[i];

                message_offset_x = (double)curr_pt_msg.offset_x + message_offset_x;
                message_offset_y = (double)curr_pt_msg.offset_y + message_offset_y;
                message_offset_z = (double)curr_pt_msg.offset_z + message_offset_z;

                tf2::Vector3 curr_pt_ecef{ecef_x + message_offset_x / 100.0, ecef_y + message_offset_y / 100.0, ecef_z + message_offset_z / 100.0}; // ecef_x is in m while message_offset_x is in cm. Want m as final result
                auto curr_pt_map = transform_to_map_frame(curr_pt_ecef);

                carma_perception_msgs::msg::PredictedState curr_state;

                if (i == 0) // First point's state should be stored outside "predictions"
                {
                    auto res = composePredictedState(curr_pt_map, prev_pt_map, out_msg.header.stamp, prev_yaw); // Position returned is that of prev_pt_map NOT curr_pt_map
                    curr_state = std::get<0>(res);
                    prev_yaw = std::get<1>(res);
                    // Compute out_msg pose
                    out_msg.pose.pose = curr_state.predicted_position;      // Orientation computed from first point in offsets with location
                    out_msg.velocity.twist = curr_state.predicted_velocity; // Velocity derived from first point
                }
                else
                {
                    rclcpp::Time updated_time_step = rclcpp::Time(prev_state.header.stamp) + rclcpp::Duration(mobility_path_prediction_time_step_ * 1e9);
                    auto res = composePredictedState(curr_pt_map, prev_pt_map, updated_time_step, prev_yaw);
                    curr_state = std::get<0>(res);
                    prev_yaw = std::get<1>(res);
                    out_msg.predictions.push_back(curr_state);
                }

                if (i == in_msg.trajectory.offsets.size() - 1) // if last point, copy the prev_state velocity & orientation to the last point too
                {
                    curr_state.predicted_position.position.x = curr_pt_map.x();
                    curr_state.predicted_position.position.y = curr_pt_map.y();
                    curr_state.predicted_position.position.z = curr_pt_map.z();
                    out_msg.predictions.push_back(curr_state);
                }

                prev_state = curr_state;
                prev_pt_map = curr_pt_map;
            }

            calculateAngVelocityOfPredictedStates(out_msg);

            return;
        }

        std::pair<carma_perception_msgs::msg::PredictedState, double> composePredictedState(const tf2::Vector3 &curr_pt, const tf2::Vector3 &prev_pt, const rclcpp::Time &prev_time_stamp, double prev_yaw) const
        {
            carma_perception_msgs::msg::PredictedState output_state;
            // Set Position
            output_state.predicted_position.position.x = prev_pt.x();
            output_state.predicted_position.position.y = prev_pt.y();
            output_state.predicted_position.position.z = prev_pt.z();

            // Set Orientation
            Eigen::Vector2d vehicle_vector = {curr_pt.x() - prev_pt.x(), curr_pt.y() - prev_pt.y()};
            Eigen::Vector2d x_axis = {1, 0};
            double yaw = 0.0;
            if (vehicle_vector.norm() < 0.000001)
            { // If there is zero magnitude use previous yaw to avoid devide by 0
                yaw = prev_yaw;
                RCLCPP_DEBUG_STREAM(logger_->get_logger(), "Two identical points sent for predicting heading. Forced to use previous yaw or 0 if first point");
            }
            else
            {
                yaw = std::acos(vehicle_vector.dot(x_axis) / (vehicle_vector.norm() * x_axis.norm()));
            }

            tf2::Quaternion vehicle_orientation;
            vehicle_orientation.setRPY(0, 0, yaw);
            output_state.predicted_position.orientation.x = vehicle_orientation.getX();
            output_state.predicted_position.orientation.y = vehicle_orientation.getY();
            output_state.predicted_position.orientation.z = vehicle_orientation.getZ();
            output_state.predicted_position.orientation.w = vehicle_orientation.getW();

            // Set velocity
            output_state.predicted_velocity.linear.x = vehicle_vector.norm() / mobility_path_prediction_time_step_;

            // Set timestamp
            output_state.header.stamp = builtin_interfaces::msg::Time(prev_time_stamp);

            return std::make_pair(output_state, yaw);
        }

        void calculateAngVelocityOfPredictedStates(carma_perception_msgs::msg::ExternalObject &object) const
        {
            if (!object.dynamic_obj)
            {
                return;
            }

            // Object's current angular velocity
            object.velocity.twist.angular.z = (getYawFromQuaternionMsg(object.pose.pose.orientation) -
                                               getYawFromQuaternionMsg(object.predictions[0].predicted_position.orientation)) /
                                              mobility_path_prediction_time_step_;

            // Predictions' angular velocities
            auto prev_orient = object.pose.pose.orientation;
            for (auto &pred : object.predictions)
            {
                pred.predicted_velocity.angular.z = (getYawFromQuaternionMsg(prev_orient) -
                                                     getYawFromQuaternionMsg(pred.predicted_position.orientation)) /
                                                    mobility_path_prediction_time_step_;
            }
        }

        double getYawFromQuaternionMsg(const geometry_msgs::msg::Quaternion &quaternion)
        {
            tf2::Quaternion orientation;
            orientation.setX(quaternion.x);
            orientation.setY(quaternion.y);
            orientation.setZ(quaternion.z);
            orientation.setW(quaternion.w);

            double roll;
            double pitch;
            double yaw;
            tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

            return yaw;
        }
    };

}
// TODO cleanup this file
