#include "unified_mocap_client.hpp"

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
##  __  __          _ _      _    ###############################################
## |  \/  |__ ___ _| (_)_ _ | |__ ##
## | |\/| / _` \ V / | | ' \| / / ##
## |_|  |_\__,_|\_/|_|_|_||_|_\_\ ##
####################################   
)" << std::endl;
    }

    ~Mocap2Mavlink()
    {
        // put cleanup here if needed
    }

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
        
        // Open UDP
        socket_fd = socket(PF_INET, SOCK_DGRAM, 0);

        if (socket_fd < 0) {
            printf("socket error: %s\n", strerror(errno));
            // return -1;
        }

        // Bind to port
        struct sockaddr_in addr = {};
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        inet_pton(AF_INET, "127.0.0.1", &(addr.sin_addr));
        addr.sin_port = htons(14551); // default port on the ground

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

        
        // std::cout << MAVLINK_MSG_ID_HEARTBEAT << std::endl;

        receive_some(socket_fd, &src_addr, &src_addr_len, &src_addr_set);

        if (src_addr_set) {
            send_some(socket_fd, &src_addr, src_addr_len);
        }
    }

    void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set)
    {
        // We just receive one UDP datagram and then return again.
        char buffer[2048]; // enough for MTU 1500 bytes

        const int ret = recvfrom(
                socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(src_addr), src_addr_len);

        if (ret < 0) {
            printf("recvfrom error: %s\n", strerror(errno));
        } else if (ret == 0) {
            // peer has done an orderly shutdown
            return;
        } 

        *src_addr_set = true;

        mavlink_message_t message;
        mavlink_status_t status;
        for (int i = 0; i < ret; ++i) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status) == 1) {

                // printf(
                //     "Received message %d from %d/%d\n",
                //     message.msgid, message.sysid, message.compid);

                switch (message.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                    handle_heartbeat(&message);
                    break;
                }
            }
        }
    }

    void handle_heartbeat(const mavlink_message_t* message)
    {
        mavlink_heartbeat_t heartbeat;
        mavlink_msg_heartbeat_decode(message, &heartbeat);

        printf("Got heartbeat from ");
        switch (heartbeat.autopilot) {
            case MAV_AUTOPILOT_GENERIC:
                printf("generic");
                break;
            case MAV_AUTOPILOT_ARDUPILOTMEGA:
                printf("ArduPilot");
                break;
            case MAV_AUTOPILOT_PX4:
                printf("PX4");
                break;
            default:
                printf("other");
                break;
        }
        printf(" autopilot\n");
    }

    void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
    {
        // Whenever a second has passed, we send a heartbeat.
        static time_t last_time = 0;
        time_t current_time = time(NULL);
        if (current_time - last_time >= 1) {
            send_heartbeat(socket_fd, src_addr, src_addr_len);
            last_time = current_time;
        }
    }

    void send_heartbeat(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
    {
        mavlink_message_t message;

        const uint8_t system_id = 42;
        const uint8_t base_mode = 0;
        const uint8_t custom_mode = 0;
        mavlink_msg_heartbeat_pack_chan(
            system_id,
            MAV_COMP_ID_PERIPHERAL,
            MAVLINK_COMM_0,
            &message,
            MAV_TYPE_GENERIC,
            MAV_AUTOPILOT_GENERIC,
            base_mode,
            custom_mode,
            MAV_STATE_STANDBY);

        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        const int len = mavlink_msg_to_send_buffer(buffer, &message);

        int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
        if (ret != len) {
            printf("sendto error: %s\n", strerror(errno));
        } else {
            printf("Sent heartbeat\n");
        }
    }

private:
    int socket_fd;
    struct sockaddr_in src_addr = {};
    socklen_t src_addr_len = sizeof(src_addr);
    bool src_addr_set = false;
};
