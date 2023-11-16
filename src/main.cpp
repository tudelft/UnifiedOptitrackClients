#include <boost/filesystem.hpp>

#include "cyberzoo_mocap_client.hpp"

#ifdef USE_CLIENT_DEBUG
    #include "debug_client.hpp"
#endif

#ifdef USE_CLIENT_IVY
    #include "ivy_client.hpp"
#endif

#ifdef USE_CLIENT_UDP
    #include "udp_client.hpp"
#endif

#if defined(USE_CLIENT_ROS2) || defined(USE_CLIENT_ROS2PX4)
    #include "ros2_client.hpp"
    #include "rclcpp/rclcpp.hpp"
    void
    h_sig_sigint( int signum )
    {
        std::cout << "Shutting down... Done. " << std::endl;
        rclcpp::shutdown();
        exit(0);
    }
#endif


int main(int argc, char const *argv[])
{
    boost::filesystem::path p(argv[0]);

#ifdef USE_CLIENT_DEBUG
    if (p.filename() == "natnet2debug") {

        DebugImpl client = DebugImpl();
        std::cout << "    ## Using client " << p.filename() << std::endl;
        client.start(argc, argv);
    } else
#endif
#ifdef USE_CLIENT_IVY
    if (p.filename() == "natnet2ivy") {
        std::cout << "    ## Using client " << p.filename() << std::endl;

        NatNet2Ivy client = NatNet2Ivy();
        client.start(argc, argv);
    } else 
#endif
#ifdef USE_CLIENT_UDP
    if (p.filename() == "natnet2udp") {
        std::cout << "    ## Using client " << p.filename() << std::endl;

        NatNet2Udp client = NatNet2Udp();
        client.start(argc, argv);
    } else 
#endif

#if defined(USE_CLIENT_ROS2) || defined(USE_CLIENT_ROS2PX4)
    
	signal(SIGINT, h_sig_sigint);

    if (p.filename() == "natnet2ros2") {
        std::cout << "    ## Using client " << p.filename() << std::endl;
        
        // Init ROS2
        rclcpp::init(argc, argv);

        // Disable Default logger
        rclcpp::get_logger("rclcpp").set_level(rclcpp::Logger::Level::Error);

        // Init Client
        NatNet2Ros2 client = NatNet2Ros2();

        // Start the thread
        client.start(argc, argv);
    } else if(p.filename() == "natnet2ros2px4") {
        std::cout << "    ## Using client " << p.filename() << std::endl;
        
        // Init ROS2
        rclcpp::init(argc, argv);

        // Disable Default logger
        rclcpp::get_logger("rclcpp").set_level(rclcpp::Logger::Level::Error);

        // Init Client
        NatNet2Ros2 client = NatNet2Ros2();

        // Start the thread
        client.start(argc, argv); 
    }
#endif
    {
        std::cout << "Support for client " << p.filename() << "was not compiled into the program." << std::endl;
        return 1;
    }

    return 0;
}

