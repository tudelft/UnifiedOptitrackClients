#include "unified_mocap_client.hpp"

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <sys/time.h>

#include <mavlink/common/mavlink.h>

class Mocap2Mavlink : public UnifiedMocapClient
{
public:
    Mocap2Mavlink()
    {
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
        close(socket_fd);
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {

    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {

    }

    void printSocketAddress(const struct sockaddr_in *addr) {
        printf("IP Address: %s\n", inet_ntoa(addr->sin_addr));
        printf("Port: %d\n", ntohs(addr->sin_port));
    }

    void pre_start() override
    {       
        // Open UDP socket
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

        receive_some(socket_fd, &src_addr, &src_addr_len, &src_addr_set);

        // print sender's address
        // printSocketAddress(&src_addr);
    }

    void publish_data() override
    {
        if (src_addr_set == true) {
            if (this->isUnpublishedRB(0)) {
                send_gps_input(socket_fd, &src_addr, src_addr_len);
                send_att_pos_mocap(socket_fd, &src_addr, src_addr_len);
            }
        }
    }

    void send_att_pos_mocap(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
    {
        mavlink_message_t message;

        uint8_t system_id = 42;
        struct timeval tv;
        uint64_t timestamp_us;
        gettimeofday(&tv, NULL);
        timestamp_us = (long long)tv.tv_sec * 1000000LL + (long long)tv.tv_usec;
        
        pose_t pose = this->getPoseRB(0);
        float q[4] = {pose.qw, pose.qx, pose.qy, pose.qz};

        float cov[21];
        cov[0] = NAN;

        printf("%ld,%f,%f,%f\n", timestamp_us, pose.x, pose.y, pose.z);

        mavlink_msg_att_pos_mocap_pack(
                system_id,
                MAV_COMP_ID_PERIPHERAL,
                &message,
                timestamp_us,
                q,
                pose.x,
                pose.y,
                pose.z,
                cov);

        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        const int len = mavlink_msg_to_send_buffer(buffer, &message);

        int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
        if (ret != len) {
            printf("sendto error: %s\n", strerror(errno));
        }
    }

    void send_gps_input(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
    {
        mavlink_message_t message;

        uint8_t system_id = 42;
        struct timeval tv;
        uint64_t timestamp_us;
        gettimeofday(&tv, NULL);
        timestamp_us = (long long)tv.tv_sec * 1000000LL + (long long)tv.tv_usec;

        mavlink_msg_gps_input_pack(
            system_id,
            MAV_COMP_ID_PERIPHERAL,
            &message,
            timestamp_us,
            0,
            254,
            0, // ms start of week
            1, // week no   
            3, // 3d fix
            386300000, // lat
            242000000, // lon
            0, // alt
            UINT16_MAX,
            UINT16_MAX,
            0,0,0,0,0,0,
            15,
            0); //yaw

        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        const int len = mavlink_msg_to_send_buffer(buffer, &message);

        int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
        if (ret != len) {
            printf("sendto error: %s\n", strerror(errno));
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

    int socket_fd;
    struct sockaddr_in src_addr = {};
    socklen_t src_addr_len = sizeof(src_addr);
    bool src_addr_set = true;
};