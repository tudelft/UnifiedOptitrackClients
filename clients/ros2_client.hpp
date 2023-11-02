#include "cyberzoo_mocap_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"

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

                geometry_msgs::msg::PoseStamped msg{};
                msg.header.stamp = this->now();
                msg.header.frame_id = "world";

                // Save in message
                msg.pose.position.x = pose.x;
                msg.pose.position.y = pose.y;
                msg.pose.position.z = pose.z;

                msg.pose.orientation.w = pose.qw;
                msg.pose.orientation.x = pose.qx;
                msg.pose.orientation.y = pose.qy;
                msg.pose.orientation.z = pose.qz;

                this->_publishers.at(i)->publish(msg);
            }
        }
      
    }

private:
    std::vector<std::string> _topics;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> _publishers;

};
