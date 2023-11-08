#include "cyberzoo_mocap_client.hpp"
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <Ivy/ivyloop.h>
#include <unistd.h>
#include <csignal>

class NatNet2Ivy : public CyberZooMocapClient
{
public:
    NatNet2Ivy()
    {
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("ac_id,ac", boost::program_options::value<unsigned int>(), "Aircraft Id to forward IVY messages to.")
            ("broadcast_address,b", boost::program_options::value<std::string>(), "Ivy broadcast ip address.")
        ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        // TODO: make this a list to correspond to rigid_body ids
        if (vm.count("ac_id"))
        {
            int val = vm["ac_id"].as<unsigned int>();
            std::cout << "Aircraft Id set to "
                      << val << std::endl;
            this->_ac_id = val;
        } else {
            std::cout << "No aircraft id passed, but is required." << std::endl;
            std::raise(SIGINT);
        }

        if (vm.count("broadcast_address"))
        {
            std::string val = vm["broadcast_address"].as<std::string>();
            std::cout << "Ivy broadcast ip set to " << val << std::endl;
            this->bip = val;
        } else {
            std::cout << "No ivy broadcast ip passed, assume 127.255.255.255" << std::endl;
            this->bip = "127.255.255.255";
        }
    }

    void pre_start() override
    {
        IvyInit ("NatNet2Ivy", "NatNet2Ivy READY", NULL, NULL, NULL, NULL);
        IvyStart(this->bip.c_str());
    }

    void post_start() override
    {
        // must be blocking!
        IvyMainLoop();
    }

    void publish_data() override
    {
        for(uint8_t i = 0; i < this->getNTrackedRB(); i++)
        {
            pose_t pose = this->getPoseRB(i);
            pose_der_t pose_der = this->getPoseDerRB(i);
            IvySendMsg("datalink EXTERNAL_POSE %d %lu  %f %f %f  %f %f %f  %f %f %f %f",
                _ac_id, pose.timeUs/1000,  //todo: probably not the right timestamp
                pose.x, pose.y, pose.z,
                pose_der.x, pose_der.y, pose_der.z,
                pose.qw, pose.qx, pose.qy, pose.qz);
        }
    }

private:
    uint8_t _ac_id;
    std::string bip;
};
