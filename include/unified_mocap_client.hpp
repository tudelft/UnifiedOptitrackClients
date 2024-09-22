// Copyright 2024 Anton Bredenbeck, Till Blaha (Delft University of Technology)
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

#ifndef H_UNIFIED_OPTITRACK_CLIENT
#define H_UNIFIED_OPTITRACK_CLIENT

#include <vector>
#include <mutex>
#include <boost/program_options.hpp>

#include <iostream>
#include <csignal>
#include <thread>

#include "mocap.hpp"
#include "agent.hpp"

char getch(void);

class UnifiedMocapClient
{
private:
    Mocap* mocap;
    Agent* agent;
    bool printMessages;
    float publish_dt;

    ArenaDirection co_north;
    float true_north_deg;
    ArenaDirection craft_nose;
    CoordinateSystem co;

    void banner();
    void add_base_po();
    void parse_base_po(int argc, char const *argv[]);
    //void print_coordinate_system() const;

    std::thread pubThread;
    std::thread keyThread;

    void keystroke_loop();

public:
    UnifiedMocapClient(Mocap* mocap, Agent* agent);
    UnifiedMocapClient(const UnifiedMocapClient &other);
    ~UnifiedMocapClient() {};

    /* Function that needs to be called after creating
     * the client to start everything */
    void start(int argc, char const *argv[]);

    boost::program_options::options_description desc;
    boost::program_options::variables_map vm;
};


#endif //H_UNIFIED_OPTITRACK_CLIENT