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

#ifdef USE_CLIENT_LOG
    #include "log_client.hpp"
#endif

#ifdef USE_CLIENT_ROS2
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
    std::cout << "    ## Using client " << p.filename() << std::endl;

    std::cout << "Attempting to start client " << p.filename() << std::endl;

    if (p.filename() == "natnet2console") {
        CyberZooMocapClient client = CyberZooMocapClient(); client.start(argc, argv);
    } else

    if (p.filename() == "natnet2console") {
        CyberZooMocapClient client = CyberZooMocapClient(); client.start(argc, argv);
    } else
#ifdef USE_CLIENT_DEBUG
    if (p.filename() == "natnet2debug") {
        NatNet2Debug client = NatNet2Debug(); client.start(argc, argv);
    } else 
#endif

#ifdef USE_CLIENT_IVY
    if (p.filename() == "natnet2ivy") {
        NatNet2Ivy client = NatNet2Ivy(); client.start(argc, argv);
    } else 
#endif

#ifdef USE_CLIENT_UDP
    if (p.filename() == "natnet2udp") {
        NatNet2Udp client = NatNet2Udp(); client.start(argc, argv);
    } else 
#endif

#ifdef USE_CLIENT_LOG
    if (p.filename() == "natnet2log") {
        NatNet2Log client = NatNet2Log(); client.start(argc, argv);
    } else 
#endif

#ifdef USE_CLIENT_ROS2
    
	signal(SIGINT, h_sig_sigint);

    if (p.filename() == "natnet2ros2") {
        // Init ROS2
        rclcpp::init(argc, argv);

        // Disable Default logger
        rclcpp::get_logger("rclcpp").set_level(rclcpp::Logger::Level::Error);

        // Init Client
        NatNet2Ros2 client = NatNet2Ros2();

        // Start the thread
        client.start(argc, argv);
    } else 
#endif

    {
        std::cout << "Support for client " << p.filename() << "was not compiled into the program." << std::endl;
        return 1;
    }

    std::cout << "Using client " << p.filename() << std::endl;
    return 0;
}

