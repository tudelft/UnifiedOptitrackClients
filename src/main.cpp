#include <boost/filesystem.hpp>

#include "unified_mocap_client.hpp"

#ifdef USE_AGENT_CONSOLE
    #include "console_agent.hpp"
#endif

#ifdef USE_AGENT_IVY
    #include "ivy_agent.hpp"
#endif

#ifdef USE_AGENT_UDP
    #include "udp_agent.hpp"
#endif

#ifdef USE_AGENT_LOG
    #include "log_agent.hpp"
#endif

#if defined(USE_AGENT_ROS2) || defined(USE_AGENT_ROS2PX4)
    #include "ros2_agent.hpp"
    #include "rclcpp/rclcpp.hpp"
#endif

#if defined(USE_AGENT_MAVLINK)
    #include "mavlink_agent.hpp"
#endif

namespace{
    std::function<void(int)> shutdown_handler;
    void signal_handler(int signal) { shutdown_handler(signal); }
}

int main(int argc, char const *argv[])
{
    boost::filesystem::path p(argv[0]);

    shutdown_handler = [p](int signum) 
    { 
        std::cout << "Shutting down... Done. " << std::endl;
#if defined(USE_AGENT_ROS2) || defined(USE_AGENT_ROS2PX4)  
        if (p.filename() == "mocap2ros2"
            || p.filename() == "mocap2ros2px4")
        {
            rclcpp::shutdown();
        }
#endif
         exit(0);
    };

	signal(SIGINT, signal_handler);

#ifdef USE_AGENT_CONSOLE
    if (p.filename() == "mocap2console") {
        Mocap2Console agent = Mocap2Console();
        agent.start(argc, argv);
    } else
#endif

#ifdef USE_AGENT_IVY
    if (p.filename() == "mocap2ivy") {
        Mocap2Ivy agent = Mocap2Ivy(); agent.start(argc, argv);
    } else 
#endif

#ifdef USE_AGENT_UDP
    if (p.filename() == "mocap2udp") {
        Mocap2Udp agent = Mocap2Udp(); agent.start(argc, argv);
    } else 
#endif

#ifdef USE_AGENT_LOG
    if (p.filename() == "mocap2log") {
        Mocap2Log agent = Mocap2Log(); agent.start(argc, argv);
    } else 
#endif

#if defined(USE_AGENT_ROS2) || defined(USE_AGENT_ROS2PX4)

    if (p.filename() == "mocap2ros2" || p.filename() == "mocap2ros2px4") {
        // Init ROS2
        rclcpp::init(argc, argv);

        // Disable Default logger
        rclcpp::get_logger("rclcpp").set_level(rclcpp::Logger::Level::Error);

        // Init agent
        Mocap2Ros2 agent = Mocap2Ros2();

        // Start the thread
        agent.start(argc, argv);
    }
#endif

#ifdef USE_AGENT_MAVLINK
    if (p.filename() == "mocap2mavlink") {
        Mocap2Mavlink agent = Mocap2Mavlink(); 
        agent.start(argc, argv);
    } else 
#endif

    {
        std::cout << "Support for agent " << p.filename() << "was not compiled into the program." << std::endl;
        return 1;
    }

    return 0;
}

