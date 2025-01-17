// Copyright 2024 Anton Bredenbeck (Delft University of Technology)
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along
// with this program. If not, see <https://www.gnu.org/licenses/>.

#ifndef ROS2_AGENT_HPP
#define ROS2_AGENT_HPP

#include "agent.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#ifdef USE_AGENT_ROS2PX4
#include <chrono>
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/timesync_status.hpp"
#endif

class ROS2Agent : public Agent, public rclcpp::Node
{
public:
    ROS2Agent() : Node("mocap_publisher",
                         rclcpp::NodeOptions().arguments(
                                {"--ros-args",
                                "--disable-rosout-logs",
                                "--log-level", "rclcpp:=error"})),
                    _topics{"/mocap"}

#ifdef USE_AGENT_ROS2PX4
                    , _px4_topics{"/fmu/in/vehicle_visual_odometry"}
#endif                    
    {
    }

    void banner() override
    {
        std::cout<< R"(
##  ___  ___  ___   ___  ##
## | _ \/ _ \/ __| |_  ) ##
## |   / (_) \__ \  / /  ##
## |_|_\\___/|___/ /___| ##
###########################)" << std::endl;
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("publish_topics,t", boost::program_options::value<std::vector<std::string> >()->multitoken(), "ROS2 topics to publish on.")
#ifdef USE_AGENT_ROS2PX4
            ("px4_ids,p", boost::program_options::value<std::vector<unsigned int> >()->multitoken(), "streaming ids to publish on px4 topics")
            ("px4_topics,o", boost::program_options::value<std::vector<std::string> >()->multitoken(), "ROS2 topics to publish the px4 msgs on")
#endif
        ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if (vm.count("publish_topics")) {
            this->_topics  = vm["publish_topics"].as<std::vector<std::string> >();
            std::cout << "ROS2 publishing topics: { ";
            for(std::string topic : this->_topics) std::cout << topic << " ";
            std::cout << "}" << std::endl;
        } else {
            std::cout << "No ROS2 Topics specified, Defaulting to: { ";
            for(std::string topic : this->_topics) std::cout << topic << " ";
            std::cout << "}" << std::endl;
        }
#ifdef USE_AGENT_ROS2PX4
        if(vm.count("px4_ids"))
        {
            this->_px4_streaming_ids = vm["px4_ids"].as<std::vector<unsigned int>>();
            std::cout << "PX4 streaming IDs set to";
            for(unsigned int id : this->_px4_streaming_ids) std::cout << " " << id << " ";
            std::cout << std::endl;
        }
        else
        {   
            // By default we take the first streaming id
            this->_px4_streaming_ids.push_back(this->getStreamingIds().at(0));

            std::cout << "PX4 Streaming IDs not set, defaulting to";
            for(unsigned int id : this->_px4_streaming_ids) std::cout << " " << id << " ";
            std::cout << std::endl;
        }

        if (vm.count("px4_topics")) {
            this->_px4_topics  = vm["px4_topics"].as<std::vector<std::string> >();
            std::cout << "PX4 publishing topics: {";
            for(std::string topic : this->_px4_topics) std::cout << " " << topic;
            std::cout << " }" << std::endl;
        } else {
            std::cout << "No PX4 Topics specified, Defaulting to: {";
            for(std::string topic : this->_px4_topics) std::cout << " " << topic;
            std::cout << " }" << std::endl;
        }
#endif
    }

    void pre_start() override
    {
        if(this->_topics.size() != this->streaming_ids.size())
        {
            std::cerr << "Number of topics does not match number of streaming ids. Exiting." << std::endl;
            std::raise(SIGINT);
        }

        for(unsigned int i = 0; i < this->_topics.size(); i++)
        {
            this->_pose_publishers.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>((this->_topics.at(i) + ("/pose")).c_str(), 10));
            this->_twist_publishers.push_back(this->create_publisher<geometry_msgs::msg::TwistStamped>((this->_topics.at(i) + ("/twist")).c_str(), 10));
        } 

#ifdef USE_AGENT_ROS2PX4

        if(this->_px4_topics.size() != this->_px4_streaming_ids.size())
        {
            std::cerr << "Number of px4 topics does not match number of px4 streaming ids. Exiting." << std::endl;
            std::raise(SIGINT);
        }
        for(unsigned int i = 0; i < this->_px4_topics.size(); i++)
        {
            this->_px4_publishers.push_back(this->create_publisher<px4_msgs::msg::VehicleOdometry>(this->_px4_topics.at(i), 10));
        }
        this->_timesync_sub = this->create_subscription<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", rclcpp::SensorDataQoS(), std::bind(&ROS2Agent::_timesync_callback, this, std::placeholders::_1));
#endif
        this->initialized = true;
    }

    bool publish_data(int idx, pose_t& pose, twist_t& twist) override
    {
        // Init Message
        geometry_msgs::msg::PoseStamped pose_msg{};
        geometry_msgs::msg::TwistStamped twist_msg{};

        // Transform the pose timestamp to unix time
        double seconds = this->time_offset;
        uint32_t seconds_t = static_cast<int32_t>(seconds);
        uint32_t nanoseconds_t = static_cast<int32_t>((seconds - seconds_t) * 1e9);
        rclcpp::Time stamp = (rclcpp::Clock(RCL_SYSTEM_TIME).now() - rclcpp::Duration(seconds_t, nanoseconds_t));

        // Init msg timestamp
        pose_msg.header.stamp = stamp;
        twist_msg.header.stamp = stamp;

        // Init Frame ID
        pose_msg.header.frame_id = "world";
        twist_msg.header.frame_id = "world";

        // Save in message
        pose_msg.pose.position.x = pose.x;
        pose_msg.pose.position.y = pose.y;
        pose_msg.pose.position.z = pose.z;

        pose_msg.pose.orientation.w = pose.qw;
        pose_msg.pose.orientation.x = pose.qx;
        pose_msg.pose.orientation.y = pose.qy;
        pose_msg.pose.orientation.z = pose.qz;

        twist_msg.twist.linear.x = twist.vx;
        twist_msg.twist.linear.y = twist.vy;
        twist_msg.twist.linear.z = twist.vz;

        twist_msg.twist.angular.x = twist.wx;
        twist_msg.twist.angular.y = twist.wy;
        twist_msg.twist.angular.z = twist.wz;

        #ifdef USE_AGENT_ROS2PX4
        /* If the current object is also in the list of ids
         * that should be published on a px4 topic */
        if(std::find(this->_px4_streaming_ids.begin(),
                        this->_px4_streaming_ids.end(),
                        streaming_ids.at(i)) != this->_px4_streaming_ids.end())
        {
            px4_msgs::msg::VehicleOdometry px4_vehicle_odometry;

            // Timestamp since system start (px4) in microseconds
            px4_vehicle_odometry.timestamp = this->_timestamp_remote + (stamp - this->_timestamp_local).nanoseconds() * 1e-3;
            px4_vehicle_odometry.timestamp_sample = px4_vehicle_odometry.timestamp;

            // Frame definition
            px4_vehicle_odometry.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

            // Pose and Twist
            /* First transform to NED */
            pose = this->toNED(pose);
            twist = this->toNED(twist);

            /* Then store */
            px4_vehicle_odometry.position = {pose.x, pose.y, pose.z};
            px4_vehicle_odometry.q = {pose.qw, pose.qx, pose.qy, pose.qz};
            px4_vehicle_odometry.velocity = {twist.vx, twist.vy, twist.vz};
            px4_vehicle_odometry.angular_velocity = {twist.wx, twist.wy, twist.wz};

            // Publish
            this->_px4_publishers.at(i)->publish(px4_vehicle_odometry);
        }
        #endif

        // Finally publish
        this->_pose_publishers.at(idx)->publish(pose_msg);
        this->_twist_publishers.at(idx)->publish(twist_msg);

        return true;
    }

private:
    std::vector<std::string> _topics;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> _pose_publishers;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> _twist_publishers;

#ifdef USE_AGENT_ROS2PX4
    std::vector<rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr> _px4_publishers;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr _timesync_sub;

    std::atomic<uint64_t> _timestamp_remote;
    rclcpp::Time _timestamp_local;

    std::vector<std::string> _px4_topics;
    std::vector<unsigned int> _px4_streaming_ids;

    //Callback function
    void _timesync_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
    {
        this->_timestamp_local = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        this->_timestamp_remote.store(msg->timestamp);
    }

    pose_t toNED(const pose_t pose) const
    {
        pose_t result = pose;

        switch(this->getCO())
        {
            case CoordinateSystem::ENU:
                result.x = pose.y;
                result.y = pose.x;
                result.z = -pose.z;

                result.qw = pose.qw;
                result.qx = pose.qy;
                result.qy = pose.qx;
                result.qz = -pose.qz;
                break;
            case CoordinateSystem::NED:
                // We do nothing because this is what we want to have
                break;
            default:
                break;
        }

        return result;
    }

    twist_t toNED(const twist_t twist) const
    {
        twist_t result = twist;

        switch(this->getCO())
        {
            case CoordinateSystem::ENU:
                result.x = twist.vy;
                result.y = twist.vx;
                result.z = -twist.vz;

                result.wx = twist.wy;
                result.wy = twist.wx;
                result.wz = -twist.wz;
                break;
            case CoordinateSystem::NED:
                // We do nothing because this is what we want to have
                break;
            default:
                break;
        }

        return result;
    }

#endif

};

#endif //ROS2_AGENT_HPP