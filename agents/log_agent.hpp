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

#ifndef LOG_AGENT_HPP
#define LOG_AGENT_HPP

#include "unified_mocap_client.hpp"

#include <iostream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#ifndef LOG_AGENT_HPP
#define LOG_AGENT_HPP
#endif // ifndef LOG_AGENT_HPP

enum LogType {CSV = 0};

std::ostream& operator<<(std::ostream& lhs, LogType e) {
    switch(e) {
    case CSV: lhs << "CSV"; break;
    }
    return lhs;
} 

class Mocap2Log : public UnifiedMocapClient
{
public:
    Mocap2Log() : _logType{LogType::CSV}
    {
        std::cout<< R"(
##  _               #############################################################
## | |   ___  __ _  ##
## | |__/ _ \/ _` | ##
## |____\___/\__, | ##
##           |___/  ##
######################
)" << std::endl;
    }

    ~Mocap2Log() {
        if (_logFile.is_open())
            _logFile.close();
    }

private:
    void add_extra_po(boost::program_options::options_description &desc) override
    {
        desc.add_options()
            ("filename,o", boost::program_options::value<std::string>(), "The filename to log to.")
            ("logtype,t", boost::program_options::value<std::string>(), "Currently only 'csv' supported.")
        ;
    }

    void parse_extra_po(const boost::program_options::variables_map &vm) override
    {
        if (vm.count("filename")) {
            this->_logFilename = vm["filename"].as<std::string>();
            std::cout << "Logfile set to " << this->_logFilename << std::endl;
        } else {
            std::cout << "Logfile argument not passed" << std::endl;
            std::raise(SIGINT);
        }

        if (vm.count("logtype")) {
            std::string val = vm["logtype"].as<std::string>();
            boost::algorithm::to_lower(val);

            if (val.compare("csv") == 0) {
                this->_logType = LogType::CSV;
            }

            std::cout << "Logging type set to " << this->_logType << std::endl;
        } else {
            std::cout << "Logging type not passed, defaulting to " 
                << this->_logType << std::endl;
        }
    }

    void pre_start() override
    {
        _logFile.open(_logFilename);
        // write header
        _logFile << "timestamp[us],RBid,x[m],y[m],z[m],qx,qy,qz,qw,vx[m/s],vy[m/s],vz[m/s],wxbody[rad/s],wybody[rad/s],wzbody[rad/s]";
        _logFile << std::endl;
    }

    void publish_data() override
    {
        for(uint8_t i = 0; i < this->getNTrackedRB(); i++)
        {
            if (this->isUnpublishedRB(i)) {
                unsigned int streaming_id = this->getStreamingId(i);
                pose_t pose = this->getPoseRB(i);
                twist_t twist = this->getPoseDerRB(i);

                _logFile << boost::format("%1%,%2%,") % pose.timeUs % streaming_id;
                _logFile << boost::format("%1%,%2%,%3%,%4%,%5%,%6%,%7%,")
                    % pose.x % pose.y % pose.z % pose.qx % pose.qy % pose.qz % pose.qw;
                _logFile << boost::format("%1%,%2%,%3%,%4%,%5%,%6%")
                    % twist.vx % twist.vy % twist.vz % twist.wx % twist.wy % twist.wz;

                _logFile << std::endl;
            }
        }
    }

private:
    LogType _logType;
    std::string _logFilename;
    std::ofstream _logFile;
};

#endif // ifndef LOG_AGENT_HPP
