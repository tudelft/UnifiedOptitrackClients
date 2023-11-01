#include "cyberzoo_mocap_client.hpp"
#include <unistd.h>
#include <csignal>
#include "boost/asio.hpp"

using namespace boost::asio;

class NatNet2Udp : public CyberZooMocapClient
{
public:
    NatNet2Udp() : _socket{_io_service}
    {
        this->_socket.open(ip::udp::v4());
    }
    ~NatNet2Udp()
    {
        this->_socket.close();
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("client_ip,i", boost::program_options::value<std::string>(), "IP to stream the UDP data to.")
            ("port,p", boost::program_options::value<unsigned short int>(), "UDP Port.")
        ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if (vm.count("client_ip")) {
            std::string val = vm["client_ip"].as<std::string>();
            std::cout << "UDP client ip " << val << std::endl;
            this->_client_ip = val;
        } else {
            std::cout << "No UDP client ip passed" << std::endl;
            std::raise(SIGINT);
        }

        if (vm.count("port")) {
            unsigned short int val = vm["port"].as<unsigned short int>();
            std::cout << "Streaming UDP on port " << val << std::endl;
            this->_port = val;
        } else {
            std::cout << "No UDP port given. Assume 5004" << std::endl;
            this->_port = 5004;
        }
    }

    void pre_start() override
    {
        this->_remote_endpoint = ip::udp::endpoint(ip::address::from_string(this->_client_ip), this->_port);
    }

    void publish_data() override
    {
        //for(uint8_t i = 0; i < this->getNTrackedRB(); i++)
        //{
            //pose_t pose = this->getPoseRB(i);
            //pose_der_t pose_der = this->getPoseDerRB(i);
            pose_t pose {.x = 1};
            pose_t pose_der {.y = 1};
            boost::system::error_code err;
            static constexpr size_t Np = sizeof(pose_t);
            static constexpr size_t Nd = sizeof(pose_der_t);
            uint8_t buf[Np+Nd];
            memcpy(buf, &pose, Np);
            memcpy(buf+Np, &pose_der, Nd);

            auto sent = this->_socket.send_to(buffer(buf, Np+Nd), this->_remote_endpoint, 0, err);
            if (err.failed())
                std::cout << "Failed with error " << err << std::endl;
        //}
    }

private:
    unsigned short int _port;
    std::string _client_ip;
    io_service _io_service;
    ip::udp::socket _socket;
    ip::udp::endpoint _remote_endpoint;
};
