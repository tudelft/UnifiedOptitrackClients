#include "unified_mocap_client.hpp"

// #include <errno.h>
// #include <stdio.h>
// #include <string.h>
// #include <stdbool.h>
// #include <stdint.h>

// #include <time.h>

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <mavlink/common/mavlink.h>

class Mocap2Mavlink : public UnifiedMocapClient
{
public:
    Mocap2Mavlink()
    {
        // ASCII art generator https://patorjk.com/software/taag/#p=display&f=Small&t=Console%20
        std::cout<< R"(
#################################################################################
####################################
##  __  __          _ _      _    ##
## |  \/  |__ ___ _| (_)_ _ | |__ ##
## | |\/| / _` \ V / | | ' \| / / ##
## |_|  |_\__,_|\_/|_|_|_||_|_\_\ ##
##                                ##
####################################   
)" << std::endl;
    }

    ~Mocap2Mavlink()
    {
        // put cleanup here if needed
    }

private:
    void add_extra_po(boost::program_options::options_description &desc) override
    {

    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {

    }

    void pre_start() override
    {
        // gets called after:
        // 1. UnifiedMocapClient is constructed
        // 2. this class is constructed
        // 3. arguments are parsed
        // 4. MocapClient and its callback handlers are constructed
        
        // Open UDP socket
        const int socket_fd = socket(PF_INET, SOCK_DGRAM, 0);

        if (socket_fd < 0) {
            printf("socket error: %s\n", strerror(errno));
            // return -1;
        }

        // Bind to port
        struct sockaddr_in addr = {};
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        inet_pton(AF_INET, "0.0.0.0", &(addr.sin_addr)); // listen on all network interfaces
        addr.sin_port = htons(14550); // default port on the ground

        if (bind(socket_fd, (struct sockaddr*)(&addr), sizeof(addr)) != 0) {
            printf("bind error: %s\n", strerror(errno));
            // return -2;
        }

        // We set a timeout at 100ms to prevent being stuck in recvfrom for too
        // long and missing our chance to send some stuff.
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
            printf("setsockopt error: %s\n", strerror(errno));
            // return -3;
        }

        struct sockaddr_in src_addr = {};
        socklen_t src_addr_len = sizeof(src_addr);
        bool src_addr_set = false;
    }

    void publish_data() override
    {
        // this gets called every this->publish_dt seconds. Publish data here.
        // for instance, this is how to just print the z-position of all tracked bodies

        // don't run anything expensive in here, just publishing

        /*
        for(uint8_t i = 0; i < this->getNTrackedRB(); i++)
        {
            if (this->isUnpublishedRB(i)) {
                unsigned int streaming_id = this->getStreamingIds()[i];

                pose_t pose = this->getPoseRB(i);
                //pose_der_t pose_der = this->getPoseDerRB(i); // derivative

                std::cout << "Rigid body with streaming id " << streaming_id 
                << " has z position " << pose.z << std::endl;
            }
        }
        */

       std::cout << "AAAAAAAAAAAAAAAAAAA" << std::endl;

    }

private:
    // can be used for extra class members
};
