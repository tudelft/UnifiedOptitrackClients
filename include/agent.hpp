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

#ifndef H_AGENT
#define H_AGENT

//#include <vector>
#include <boost/program_options.hpp>

#include <iostream>
#include <csignal>
#include <thread>

#include "pose_calculations.hpp"
#include "rigid_body.hpp"

class Agent
{
private:
protected:
    bool printMessages;
    unsigned int publish_every;
    CoordinateSystem csys;
    ArenaDirection north_dir;
    float true_north_deg;
public:
    Agent();
    Agent(const Agent &other);
    ~Agent();

    void set_publish_divisor( unsigned int div );
    void set_csys( CoordinateSystem csys );
    void set_north( ArenaDirection dir, float true_north_deg);
    void new_data_available( std::vector<RigidBody>& RBs );

    virtual void banner();

    /* Virtual Function to be implemented by base classes */
    // called just before/after start of the publishing thread
    virtual void pre_start(); 
    virtual void post_start(); // must be blocking
    // Extra Program Options
    virtual void add_extra_po(boost::program_options::options_description &desc);
    virtual void parse_extra_po(const boost::program_options::variables_map &vm);

    // Publishing functions
    virtual void print_data(int idx, pose_t& pose, twist_t& twist);
    virtual bool publish_data(int idx, pose_t& pose, twist_t& twist);

    inline void togglePrintMessages() { printMessages ^= true; };
};


#endif //H_AGENT