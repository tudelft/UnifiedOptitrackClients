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

#include "agent.hpp"

Agent::Agent() : printMessages{false}
{
}

Agent::Agent(const Agent &other)
{
    (void) other;
    std::cerr << "Copy constructor for Agent not supported. Exiting." << std::endl;
    std::raise(SIGINT);
}

Agent::~Agent()
{
}

void Agent::banner()
{
}

void Agent::print_data(int idx, pose_t& pose, pose_der_t& pose_der)
{
    if ( this->printMessages ) {
        //printf("Incoming Rigid Body Data Frame [ID=%d Error=%3.4f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
        //printf("\t\tx\ty\tz\tqx\tqy\tqz\tqw\n");
        //printf("Incoming: \t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\n",
        //    data->RigidBodies[i].x,
        //    data->RigidBodies[i].y,
        //    data->RigidBodies[i].z,
        //    data->RigidBodies[i].qx,
        //    data->RigidBodies[i].qy,
        //    data->RigidBodies[i].qz,
        //    data->RigidBodies[i].qw);
        printf("Published: \t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\n",
            pose.x,
            pose.y,
            pose.z,
            pose.qx,
            pose.qy,
            pose.qz,
            pose.qw);
        printf("\t\tvx\tvy\tvz\twx\twy\twz\n");
        printf("Published: \t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\n\n",
            pose_der.x,
            pose_der.y,
            pose_der.z,
            pose_der.wx,
            pose_der.wy,
            pose_der.wz);
    }
}

// Non-action implementation of the virtual function to make the implementation optional
bool Agent::publish_data(int idx, pose_t& pose, pose_der_t& pose_der)
{
    return false;
}

// Non-action implementation of the virtual function to make the implementation optional
void Agent::pre_start()
{
}

// Non-action implementation of the virtual function to make the implementation optional
void Agent::post_start()
{
}

// Non-action implementation of the virtual function to make the implementation optional
void Agent::add_extra_po(boost::program_options::options_description &desc)
{
    (void)desc;
}

// Non-action implementation of the virtual function to make the implementation optional
void Agent::parse_extra_po(const boost::program_options::variables_map &vm)
{
    (void)vm;
}

