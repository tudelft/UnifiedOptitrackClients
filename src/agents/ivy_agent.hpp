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

#ifndef IVY_AGENT_HPP
#define IVY_AGENT_HPP

#include "agent.hpp"

#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <Ivy/ivyloop.h>
#include <unistd.h>
#include <csignal>

class IvyAgent : public Agent
{
public:
    IvyAgent()
    {
    }

    ~IvyAgent()
    {
        IvyStop();
    }

    void banner() override
    {
        // ASCII art generator https://patorjk.com/software/taag/#p=display&f=Small&t=Console%20
        std::cout<< R"(
##  ___           ##
## |_ _|_ ___  _  ##
##  | |\ V / || | ##
## |___|\_/ \_, | ##
##          |__/  ##
####################)" << std::endl;
    }

    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("ac_id,ac", boost::program_options::value<std::vector<unsigned int>>()->multitoken(), "Aircraft Id(s) to forward IVY messages to.")
            ("broadcast_address,b", boost::program_options::value<std::string>(), "Ivy broadcast ip address.")
        ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if (vm.count("ac_id"))
        {
            this->_ac_id = vm["ac_id"].as<std::vector<unsigned int>>();
            if (this->_ac_id.size() != this->streaming_ids.size()) {
                std::cout << "Number of ac_ids and streaming_ids passed must be equal"
                    << std::endl;
                std::raise(SIGINT);
            }
            std::cout << "AC IDs set to";
            for(unsigned int id : this->_ac_id) std::cout << " " << id << " ";
            std::cout << std::endl;
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
        IvyInit ("Mocap2Ivy", "Mocap2Ivy READY", NULL, NULL, NULL, NULL);
        IvyStart(this->bip.c_str());
        this->initialized = true;
    }

    void post_start() override
    {
        // must be blocking!
        IvyMainLoop();
    }

    bool publish_data(int idx, pose_t& pose, twist_t& twist) override
    {
        IvySendMsg("datalink EXTERNAL_POSE %d %lu  %f %f %f  %f %f %f  %f %f %f %f",
            _ac_id[idx], pose.timeUs/1000,  //todo: probably not the right timestamp
            pose.x, pose.y, pose.z,
            twist.vx, twist.vy, twist.vz,
            pose.qw, pose.qx, pose.qy, pose.qz);

        return true;
    }

private:
    std::vector<unsigned int> _ac_id;
    std::string bip;
};

#endif // ifndef IVY_AGENT_HPP
