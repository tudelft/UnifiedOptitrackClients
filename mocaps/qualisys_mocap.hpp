// Copyright 2024 Till Blaha (Delft University of Technology)
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

#ifndef H_QUALISYS_MOCAP
#define H_QUALISYS_MOCAP

#include <vector>
#include <mutex>
#include <boost/program_options.hpp>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>

#include <iostream>
#include <csignal>
#include <thread>

#include <unistd.h>
#include <termios.h>

#include "pose_calculations.hpp"

namespace po = boost::program_options;

//using namespace boost::asio;

enum QualisysType {
    Error = 0,
    Command,
    XML,
    Data,
    NoMoreData,
    C3Cfile,
    Event,
    Discover,
    QTMfile
};

enum QualisysComponent {
    _6D = 5
};

// todo: gcc packing?
typedef struct qualisys_header_s {
    unsigned int Size;
    unsigned int Type;
} qualisys_header_t;

typedef struct qualisys_packet_s {
    qualisys_header_t header;
    char* data;
} qualisys_packet_t;

// udp streaming packet header
typedef struct qualisys_data_header_s {
    unsigned int Size;
    unsigned int Type;
    unsigned long long MarkerTimestamp;
    unsigned int MarkerFrameNumber;
    unsigned int ComponentCount;
} qualisys_data_header_t;

// 6DOF components
typedef struct qualisys_6dof_header_s {
    unsigned int ComponentSize;
    unsigned int ComponentType;
    unsigned int BodyCount;
    unsigned short dropRate;
    unsigned short outOfSyncRate;
} qualisys_6dof_header_t;

typedef struct qualisys_6dof_body_s {
    float x;  // position
    float y;
    float z;
    float r00; // rotation matrix
    float r01;
    float r02;
    float r10;
    float r11;
    float r12;
    float r20;
    float r21;
    float r22;
} qualisys_6dof_body_t;



class QualisysMocap : public Mocap
{
public:
    QualisysMocap() : Mocap{}, socket_udp{io_context_udp}
    {
        // setup work, but do as little as possible
    }

    ~QualisysMocap()
    {
        // cleanup if necessary
        this->io_context_udp.stop();
        this->asyncReceiver.join();
    }

    void banner() override
    {
        // ASCII art generator https://patorjk.com/software/taag/#p=display&f=Small&t=Console%20
        std::cout<< R"(
##   ___            _ _             ##
##  / _ \ _  _ __ _| (_)____  _ ___ ##
## | (_) | || / _` | | (_-< || (_-< ##
##  \__\_\\_,_\__,_|_|_/__/\_, /__/ ##
##                         |__/     ##
######################################)";
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("mocap_ip", boost::program_options::value<std::string>(), "QTM IP.")
            ("mocap_tcp_port", boost::program_options::value<unsigned short int>(), "QTM TCP port.")
            ("mocap_udp_port", boost::program_options::value<unsigned short int>(), "Arbitrary UDP port 1024-65535 for streaming.")
            ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if (vm.count("mocap_ip")) {
            std::string val = vm["mocap_ip"].as<std::string>();
            std::cout << "QTM server ip " << val << std::endl;
            this->mocap_ip = val;
        } else {
            std::cout << "No QTM server ip passed" << std::endl;
            std::raise(SIGINT);
        }

        if (vm.count("mocap_tcp_port")) {
            std::string val = vm["mocap_tcp_port"].as<std::string>();
            std::cout << "Connecting to QTM on TCP port " << val << std::endl;
            this->mocap_tcp_port = val;
        } else {
            std::cout << "No QTM TCP port given. Assume 22223 (little-endian)" << std::endl;
            this->mocap_tcp_port = "22223";
        }

        if (vm.count("mocap_udp_port")) {
            unsigned short int val = vm["mocap_udp_port"].as<unsigned short int>();
            std::cout << "Telling QTM to stream on UDP port " << val << std::endl;
            this->mocap_udp_port = val;
        } else {
            std::cout << "No QTM udp port given. Assume 12345" << std::endl;
            this->mocap_udp_port = 12345;
        }
    }

    int connect() override
    {
        // will be called the first thing after all options are parsed
        // should be used to establish connection with the MOCAP system, and
        // configure it if necessary

        // IMPORTANT: 
        // every time a new sample comes in, the base class method 
        // "processNewPose" must be invoked.
        // in this test example, we do this at the end of the send_test_data
        // thread.
        // also see the optitrack client, which does it with callback

        boost::asio::ip::tcp::resolver resolver(this->io_service_tcp);
        this->socket_tcp = new boost::asio::ip::tcp::socket(this->io_service_tcp);
        boost::asio::ip::tcp::resolver::query query(this->mocap_ip, this->mocap_tcp_port);
        boost::asio::connect(*this->socket_tcp, resolver.resolve(query));
        std::cout << "Connected to Qualisys server." << std::endl;


        this->send_tcp_command("Version");
        this->send_tcp_command("StreamFrames AllFrames 6D FrequencyDivisor:5 UDP:12345");

        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        this->remote_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), this->mocap_udp_port);
        this->socket_udp.open(boost::asio::ip::udp::v4());
        this->socket_udp.bind(this->remote_endpoint);

        this->asyncReceiver = std::thread([this]() {
            io_context_udp.run(); // This will block and process async events
        });
        this->wait();

        return 0;
    }

    void data_handler(const boost::system::error_code& error, std::size_t bytes_transferred)
    {
        char* buf = recv_buffer.data();

        qualisys_data_header_t header;
        memcpy(&header, buf, sizeof(qualisys_data_header_t));
        buf += sizeof(qualisys_data_header_t);

        if (header.Type != QualisysType::Data) { goto bail_out; }

        for (int i=0; i < header.ComponentCount; i++) {
            //std::cout << i << std::endl;
            qualisys_6dof_header_t header6d;
            memcpy(&header6d, buf, sizeof(qualisys_6dof_header_t));
            buf += sizeof(qualisys_6dof_header_t);

            if (header6d.ComponentType != QualisysComponent::_6D) {
                continue;
            }

            for (int j = 0; j < header6d.BodyCount; j++) {
                qualisys_6dof_body_t body;
                memcpy(&body, buf, sizeof(qualisys_6dof_body_t));
                buf += sizeof(qualisys_6dof_body_t);

                quaternion_t quat;
                rotationMatrix_t rotM = { .m = {
                        { body.r00, body.r01, body.r02 },
                        { body.r10, body.r11, body.r12 },
                        { body.r20, body.r21, body.r22 },
                }};

                quaternion_of_rotationMatrix(&quat, &rotM);

                pose_t newPose = { header.MarkerTimestamp,
                    body.x, body.y, body.z,
                    quat.x, quat.y, quat.z, quat.w
                    };

                if (this->RBs.size() > 0) {
                    this->RBs[0].setNewPoseENU( newPose );
                }

                // only the first body gets read
                i = header.ComponentCount;
                break;
            }
        }

        this->agent->new_data_available( this->RBs );

bail_out:
        //std::cout << header.Type << std::endl;
        wait();
    }

    void wait() {
        this->socket_udp.async_receive_from(boost::asio::buffer(this->recv_buffer),
            this->remote_endpoint,
            boost::bind(&QualisysMocap::data_handler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }


    void send_tcp_command(const std::string& command)
    {
        qualisys_header_t header;
        header.Size = command.length() + sizeof(qualisys_header_t);
        header.Type = QualisysType::Command;

        write(*this->socket_tcp, boost::asio::buffer(&header, sizeof(qualisys_header_t)));
        write(*this->socket_tcp, boost::asio::buffer(command));

        std::cout << "Sent command: " << command << std::endl;
    }

private:
    std::string mocap_ip;
    std::string mocap_tcp_port;
    boost::asio::io_service io_service_tcp;
    boost::asio::ip::tcp::socket* socket_tcp;

    unsigned short int mocap_udp_port;
    boost::asio::io_context io_context_udp;
    boost::asio::ip::udp::socket socket_udp;

    boost::asio::ip::udp::endpoint remote_endpoint;
    boost::array<char, 4096> recv_buffer;

    std::thread asyncReceiver;
};

#endif // H_QUALISYS_MOCAP