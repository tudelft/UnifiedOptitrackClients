#ifndef ROS2_CLIENT_HPP
#define ROS2_CLIENT_HPP

#include "cyberzoo_mocap_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#ifdef USE_CLIENT_ROS2PX4
#include <chrono>
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/timesync_status.hpp"
#endif

class NatNet2Ros2 : public CyberZooMocapClient, public rclcpp::Node
{
public:
    NatNet2Ros2() : Node("mocap_publisher",
                         rclcpp::NodeOptions().arguments(
                                {"--ros-args",
                                "--disable-rosout-logs",
                                "--log-level", "rclcpp:=error"})),
                    _topics{"/mocap"}
    {  
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("publish_topics, t", boost::program_options::value<std::vector<std::string> >()->multitoken(), "ROS2 topics to publish on.")
#ifdef USE_CLIENT_ROS2PX4
            ("px4_ids, p", boost::program_options::value<std::vector<unsigned int> >()->multitoken(), "streaming ids to publish on px4 topics")
#endif
        ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if (vm.count("publish_topics")) {
            this->_topics  = vm["publish_topic"].as<std::vector<std::string> >();
            std::cout << "ROS2 publishing topics: {";
            for(std::string topic : this->_topics) std::cout << " " << topic;
            std::cout << " }" << std::endl;
        } else {
            std::cout << "No ROS2 Topics specified, Defaulting to: {";
            for(std::string topic : this->_topics) std::cout << " " << topic;
            std::cout << " }" << std::endl;
        }
#ifdef USE_CLIENT_ROS2PX4
        if(vm.count("px4_ids"))
        {
            this->_px4_streaming_ids = vm["px4_ids"].as<std::vector<unsigned int>>();
            std::cout << "PX4 streaming IDs set to";
            for(unsigned int id : this->_px4_streaming_ids) std::cout << " " << id << " ";
            std::cout << std::endl;

            if(this->_px4_streaming_ids.size() > 1)
            {
                std::cerr << "Currently not supporting multiple PX4 vehicles. Shutting down..." << std::endl;
                std::raise(SIGINT);
            }
        }
        else
        {   
            // By default we take the first streaming id
            this->_px4_streaming_ids.push_back(this->get_streaming_ids().at(0));

            std::cout << "PX4 Streaming IDs not set, defaulting to";
            for(unsigned int id : this->_px4_streaming_ids) std::cout << " " << id << " ";
            std::cout << std::endl;
        }
#endif
    }

    void pre_start() override
    {
        if(this->_topics.size() != this->get_streaming_ids().size())
        {
            std::cerr << "Number of topics does not match number of streaming ids. Exiting." << std::endl;
            std::raise(SIGINT);
        }

        for(unsigned int i = 0; i < this->_topics.size(); i++)
        {
            this->_publishers.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(this->_topics.at(i).c_str(), 10));
        } 

#ifdef USE_CLIENT_ROS2PX4
        this->_px4_publisher = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);
        this->_timesync_sub = this->create_subscription<px4_msgs::msg::TimesyncStatus>("/fmu/out/timesync_status", rclcpp::SensorDataQoS(), std::bind(&NatNet2Ros2::_timesync_callback, this, std::placeholders::_1));
#endif
    }

    void publish_data() override
    {
        for(unsigned int i = 0; i < this->_publishers.size(); i++)
        {   
            // Only publish messages if the current value is valid
            if(this->getValidRB(i))
            {
                // Get the current pose
                pose_t pose = this->getPoseRB(i);

                // Init Message
                geometry_msgs::msg::PoseStamped msg{};
                
                // Transform the pose timestamp to unix time
                double seconds = this->seconds_since_mocap_ts(pose.timeUs);
                uint32_t seconds_t = static_cast<int32_t>(seconds);
                uint32_t nanoseconds_t = static_cast<int32_t>((seconds - seconds_t) * 1e9);
                msg.header.stamp = (this->now() - rclcpp::Duration(seconds_t, nanoseconds_t));

                // Init Frame ID
                msg.header.frame_id = "world";

                // Save in message
                msg.pose.position.x = pose.x;
                msg.pose.position.y = pose.y;
                msg.pose.position.z = pose.z;

                msg.pose.orientation.w = pose.qw;
                msg.pose.orientation.x = pose.qx;
                msg.pose.orientation.y = pose.qy;
                msg.pose.orientation.z = pose.qz;

                // Finally publish
                this->_publishers.at(i)->publish(msg);
            }
        }
      
    }

private:
    std::vector<std::string> _topics;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> _publishers;

#ifdef USE_CLIENT_ROS2PX4
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr _timesync_sub;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr _px4_publisher;

    std::atomic<uint64_t> _timestamp_remote;
    std::chrono::time_point<std::chrono::steady_clock> _timestamp_local;

    std::vector<unsigned int> _px4_streaming_ids;

    //Callback function
    void _timesync_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
    {
        this->_timestamp_local = std::chrono::steady_clock::now();
        this->_timestamp_remote.store(msg->timestamp);
    }

#endif

};

#endif //ROS2_CLIENT_HPP