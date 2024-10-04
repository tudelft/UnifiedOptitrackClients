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

#include "unified_mocap_client.hpp"

#include <iostream>
#include <iomanip>
#include <string>
#include <thread>
#include <boost/algorithm/string.hpp>
#include <functional>

#include <unistd.h>
#include <termios.h>
#include <inttypes.h>

#include "mocap.hpp"
#include "agent.hpp"

namespace po = boost::program_options;

// stream operators for enums
/*
std::ostream& operator<<(std::ostream& lhs, ErrorCode e) {
    switch(e) {
    case ErrorCode_OK: lhs <<               "OK"; break;
    case ErrorCode_Internal: lhs <<         "Internal"; break;
    case ErrorCode_External: lhs <<         "External"; break;
    case ErrorCode_Network: lhs <<          "Network"; break;
    case ErrorCode_Other: lhs <<            "Other"; break;
    case ErrorCode_InvalidArgument: lhs <<  "InvalidArgument"; break;
    case ErrorCode_InvalidOperation: lhs << "InvalidOperation"; break;
    case ErrorCode_InvalidSize: lhs <<      "InvalidSize"; break;
    }
    return lhs;
} 

std::ostream& operator<<(std::ostream& lhs, CoordinateSystem e) {
    switch(e) {
    case UNCHANGED: lhs << "UNCHANGED"; break;
    case NED: lhs << "NED"; break;
    case ENU: lhs << "ENU"; break;
    }
    return lhs;
} 

std::ostream& operator<<(std::ostream& lhs, ArenaDirection e) {
    switch(e) {
    case RIGHT: lhs << "RIGHT"; break;
    case FAR_SIDE: lhs << "FAR_SIDE"; break;
    case LEFT: lhs << "LEFT"; break;
    case NEAR_SIDE: lhs << "NEAR_SIDE"; break;
    }
    return lhs;
} 

std::ostream& operator<<(std::ostream& lhs, UpAxis e) {
    switch(e) {
    case NOTDETECTED: lhs << "NOTDETECTED"; break;
    case X: lhs << "X"; break;
    case Y: lhs << "Y"; break;
    case Z: lhs << "Z"; break;
    }
    return lhs;
}
*/

UnifiedMocapClient::UnifiedMocapClient(const UnifiedMocapClient &other)
{
    (void) other;
    std::cerr << "Copy constructor for UnifiedMocapClient not supported. Exiting." << std::endl;
    std::raise(SIGINT);
}

UnifiedMocapClient::UnifiedMocapClient(Mocap* mocap, Agent* agent) :
    printMessages{false}, publish_dt{0.01f}, desc{"Allowed options"}, vm{}, publish_div{0}
{
    // TODO: use builtin forward prediction with the latency estimates plus a 
    // user-defined interval (on the order of 10ms)?
    this->mocap = mocap;
    this->agent = agent;
    this->mocap->enroll_agent(this->agent);

    this->banner();
    this->mocap->banner();
    this->agent->banner();
}

/*
void UnifiedMocapClient::publish_loop()
{
    using namespace std::chrono_literals;
    bool run = true;
    auto ts = std::chrono::steady_clock::now();
    float sleep_time = this->publish_dt;

    while(run)
    {
        if (this->testMode)
        {
            sFrameOfMocapData fakeData;
            const auto microsecondsSinceEpoch = std::chrono::time_point_cast<std::chrono::microseconds>(ts).time_since_epoch().count();
            fakeData.CameraMidExposureTimestamp = microsecondsSinceEpoch;
            fakeData.nRigidBodies = 1;
            fakeData.RigidBodies[0].ID = this->getStreamingIds()[0];
            fakeData.RigidBodies[0].MeanError = 0.001;
            fakeData.RigidBodies[0].qw = 1.;
            fakeData.RigidBodies[0].qx = 0.;
            fakeData.RigidBodies[0].qy = 0.;
            fakeData.RigidBodies[0].qz = 0.;
            fakeData.RigidBodies[0].x = 1.;
            fakeData.RigidBodies[0].y = 2.;
            fakeData.RigidBodies[0].z = 3.;
            fakeData.RigidBodies[0].params = 0x01; // valid
            this->natnet_data_handler(&fakeData);
        }

        // Transform the pose to the desired coordinate system
        pose_t newPose = transform_pose(this->co,
                                this->co_north,
                                this->true_north_deg,
                                this->up_axis,
                                this->long_edge,
                                this->craft_nose,
                                newPose);

        this->publish_data();
        this->setPublishedAllRB();

        if (this->printMessages) {
		    printf("Incoming Rigid Body Data Frame [ID=%d Error=%3.4f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
		    printf("\t\tx\ty\tz\tqx\tqy\tqz\tqw\n");
		    printf("Incoming: \t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\n",
		    	data->RigidBodies[i].x,
		    	data->RigidBodies[i].y,
		    	data->RigidBodies[i].z,
		    	data->RigidBodies[i].qx,
		    	data->RigidBodies[i].qy,
		    	data->RigidBodies[i].qz,
		    	data->RigidBodies[i].qw);
		    printf("Published: \t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\n",
		    	newPose.x,
		    	newPose.y,
		    	newPose.z,
		    	newPose.qx,
		    	newPose.qy,
		    	newPose.qz,
		    	newPose.qw);
            printf("\t\tvx\tvy\tvz\twx\twy\twz\n");
		    printf("Published: \t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\t%+3.3f\n\n",
                newPoseDer.x,
                newPoseDer.y,
                newPoseDer.z,
                newPoseDer.wx,
                newPoseDer.wy,
                newPoseDer.wz);
        }

        using namespace std::chrono_literals;
        std::this_thread::sleep_for(sleep_time * 1s);

        // We measure the duration of publish_data and adjust 
        // the publish_dt based on that in a closed loop fashion
        // in order to achieve the desired frequency
        auto ts_new = std::chrono::steady_clock::now();
        std::chrono::duration<float> duration = ts_new - ts;

        sleep_time += 0.25 * (this->publish_dt - duration.count()); 

        ts = ts_new;
    }
}
*/

void UnifiedMocapClient::keystroke_loop()
{
    // wait for keystrokes
    std::cout << std::endl << "Listening to messages! Press q to quit, Press t to toggle message printing" << std::endl;

	while ( const int c = getch() )
    {
        switch(c)
        {
            case 'q':
                std::raise(SIGINT);
                break;
            case 't':
                this->agent->togglePrintMessages();
                break;
        }
    }
}

void UnifiedMocapClient::start(int argc, const char *argv[])
{
    this->add_base_po();
    this->mocap->add_extra_po(this->desc);
    this->agent->add_extra_po(this->desc);

    this->parse_base_po(argc, argv);
    this->mocap->parse_extra_po(this->vm);
    this->agent->parse_extra_po(this->vm);

    this->mocap->connect();

    //this->print_coordinate_system();

    // initialize filters for derivatives
    //for (unsigned int i=0; i < MAX_TRACKED_RB; i++) {
    //    derFilter[i] = PoseDifferentiator();
    //}

    this->agent->pre_start();
    //this->pubThread = std::thread(&UnifiedMocapClient::publish_loop, this);
    this->keyThread = std::thread(&UnifiedMocapClient::keystroke_loop, this);
    this->agent->post_start();

    // some blocking code
    //this->pubThread.join();
    this->keyThread.join();
}

void UnifiedMocapClient::add_base_po()
{
    this->desc.add_options()
        ("help,h", "produce help message")
        ("publish_divisor,d", po::value<unsigned int>(), "Publish every <publish_divisor> sample of the incoming MoCap data")
        ("publish_frequency,f", po::value<float>(), "Current not implemented. May be alternative to -d.")
        ("coordinate_system,c", po::value<std::string>(), "coordinate system convention to use [ned, enu]")
        ("coordinate_north,r", po::value<std::string>(), "where north should be relative to the observer. Either non-zero number describing right-hand rotation of north axis from the far side, or one of [right, far_side, left, near_side].")
        ("streaming_ids,s", po::value<std::vector<unsigned int>>()->multitoken(), "streaming ids to track")
        ("craft_noses,n", po::value<std::vector<std::string>>()->multitoken(), "direction of aircraft noses when creating the rigid body in the mocap software. space-separated list of [right, far_side, left, near_side]")
    ;
}

void UnifiedMocapClient::parse_base_po(int argc, char const *argv[])
{
    po::store(po::parse_command_line(argc, argv, this->desc), this->vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        exit(0);
    }

    if(vm.count("coordinate_system"))
    {
        std::string co_name = vm["coordinate_system"].as<std::string>();
        boost::algorithm::to_lower(co_name);

        //if(co_name.compare("unchanged") == 0) { this->co = CoordinateSystem::UNCHANGED; }
        //else
        if(co_name.compare("ned") == 0) { this->co = CoordinateSystem::NED; }
        else if (co_name.compare("enu") == 0) { this->co = CoordinateSystem::ENU; }
        else {
            std::cout << "Coordinate system " << co_name << " not definied. Exiting" << std::endl;
            std::raise(SIGINT);
        }
        std::cout << "Coordinate system set to " << this->co << std::endl;
    }
    else
    {
        std::cout << "Coordinate System not set, defaulting to "
                  << this->co << std::endl;
    }

    if(vm.count("coordinate_north"))
    {
        //if (this->co == CoordinateSystem::UNCHANGED) {
        //    std::cout << "Can only specify --coordinate_north/-r when coordinate system is not UNCHANGED. Exiting" << std::endl;
        //    std::raise(SIGINT);
        //}
        std::string co_north = vm["coordinate_north"].as<std::string>();
        boost::algorithm::to_lower(co_north);

        if(co_north.compare("right") == 0)
        {
            this->co_north = ArenaDirection::RIGHT;
        }
        else if(co_north.compare("far_side") == 0)
        {
            this->co_north = ArenaDirection::FAR_SIDE;
        }
        else if (co_north.compare("left") == 0)
        {
            this->co_north = ArenaDirection::LEFT;
        }
        else if (co_north.compare("near_side") == 0)
        {
            this->co_north = ArenaDirection::NEAR_SIDE;
        }
        else
        {
            this->co_north = ArenaDirection::TRUE_NORTH;
            this->true_north_deg = std::atof(co_north.c_str());
            if (this->true_north_deg == 0.0) {
                std::cout << "Coordinate system argument " << co_north << " is neither [near_side, far_side, right, left], nor float (for 0.0 use far_side). Exiting" << std::endl;
                std::raise(SIGINT);
            }
        }

        std::cout << "Coordinate system north set to " << this->co_north << std::endl;
    }
    else
    {
        std::cout << "Coordinate System north not set, defaulting to "
                  << this->co_north << std::endl;
    }

    if (vm.count("publish_frequency") + vm.count("publish_divisor") != 1) {
        std::cout << "must pass either --publish_frequency/-f or --publish_divisor/-d" << std::endl;
        exit(1);
    }

    if (vm.count("publish_frequency")) {
        std::cout << "Not implemented, use --publish_divisor/-d instead. sorry" << std::endl;
        exit(1);

        this->publish_dt = 1.0 / vm["publish_frequency"].as<float>();
        std::cout << "Publish frequency was set to " 
                  << 1.0 / this->publish_dt << " Hz" << std::endl;
    } else if (vm.count("publish_divisor")) {
        this->publish_div = vm["publish_divisor"].as<unsigned int>();
        if (this->publish_div == 0) {
            std::cout << "publish_divisor must be greater than 0" << std::endl;
            exit(1);
        }
        std::cout << "Publishing every "
                  << this->publish_div << " th sample that will be received." << std::endl;
    }

    if(vm.count("streaming_ids"))
    {
        this->streaming_ids = vm["streaming_ids"].as<std::vector<unsigned int>>();
        //std::cout << "Streaming IDs set to";
        //for(unsigned int id : this->streaming_ids) std::cout << " " << id << " ";
        //std::cout << std::endl;
    }
    else
    {
        std::cout << "Streaming IDs not set" <<std::endl;
        exit(1);
    }

    if(vm.count("craft_noses"))
    {
        this->craft_nose_strings = vm["craft_noses"].as<std::vector<std::string>();
        if (this->craft_nose_strings.size() != this->streaming_ids.size()) {
            std::cout << "Streaming IDs input must be the same length as craft_noses!" << std::endl;
        }

        for (size_t i = 0; i < this->craft_nose_strings.size(); ++i) {
            std::string str = this->craft_nose_strings[i];
            boost::algorithm::to_lower(str);

            ArenaDirection dir;
            if(str.compare("right") == 0)
            {
                dir = ArenaDirection::RIGHT;
            }
            else if(str.compare("far_side") == 0)
            {
                dir = ArenaDirection::FAR_SIDE;
            }
            else if (str.compare("left") == 0)
            {
                dir = ArenaDirection::LEFT;
            }
            else if (str.compare("near_side") == 0)
            {
                dir = ArenaDirection::NEAR_SIDE;
            }
            else
            {
                std::cout << "A/C Nose Direction " << str << " not definied. Exiting" << std::endl;
                std::raise(SIGINT);
            }

            std::cout << "Creating rigid body with streaming id " << this->streaming_ids[i] << " and nose direction " << dir << std::endl;
            this->trackedRBs.push_back(new RigidBody( dir ));
        }
    }
    else
    {
        std::cout << "A/C nose direction not set, exiting" << std::endl;
        std::raise(SIGINT);
    }
}

void UnifiedMocapClient::banner()
{
    // generator and font: https://patorjk.com/software/taag/#p=display&f=Small&t=Type%20Something%20
    std::cout<< R"(
#################################################################################
##  _   _      _  __ _        _ __  __                    ___ _ _         _    ##
## | | | |_ _ (_)/ _(_)___ __| |  \/  |___  __ __ _ _ __ / __| (_)___ _ _| |_  ##
## | |_| | ' \| |  _| / -_) _` | |\/| / _ \/ _/ _` | '_ \ (__| | / -_) ' \  _| ##
##  \___/|_||_|_|_| |_\___\__,_|_|  |_\___/\__\__,_| .__/\___|_|_\___|_||_\__| ##
##                                                 |_|                         ##
#################################################################################)";
}

/*
void UnifiedMocapClient::print_coordinate_system() const
{
    // There's probably a better way to do this but I can't think
    // of it. So a bunch of nested switch-case statements it is.
    std::cout<< R"( 
               far
     +──────────────────────────+
     │                          │)";
    switch(this->craft_nose)
    {
        case ArenaDirection::RIGHT:
            std::cout << R"(
     │                          │
     │       [craft] ▶          │
     │                          │)";
            break;
        case ArenaDirection::FAR_SIDE:
            std::cout << R"(
     │            ▲             │
     │         [craft]          │
     │                          │)";
            break;
        case ArenaDirection::LEFT:
            std::cout << R"(
     │                          │
     │       ◀ [craft]          │
     │                          │)";
            break;
        case ArenaDirection::NEAR_SIDE:
            std::cout << R"(
     │                          │
     │         [craft]          │
     │            ▼             │)";
            break;
    }

    std::cout << R"(
left │                          │ right )";

    // If the up_axis is not detected the CO is not well defined
    // and we can't draw the coordinate system
    if(this->up_axis == UpAxis::NOTDETECTED)
    {
        std::cout << R"(
     │ UpAxis could not be      │
     │ detected. CO not well    │ 
     │ defined. Use a newer     │ 
     │ version of motive.       │)" << std::endl;
        return;
    }
    switch(this->co)
    {
        case CoordinateSystem::UNCHANGED:
            switch(this->up_axis)
            {
                case UpAxis::X:
                    std::cout << R"( 
     │      ↑                   │
     │    x ⊙ →                 │
     │                          │
     │    y-z Unchanged         │)";
                    break;
                case UpAxis::Y:
                    std::cout << R"( 
     │      ↑                   │
     │    y ⊙ →                 │
     │                          │
     │    x-z Unchanged         │)";
                    break;
                case UpAxis::Z:
                    std::cout << R"( 
     │      ↑                   │
     │    z ⊙ →                 │
     │                          │
     │    x-y Unchanged         │)";
                    break;
            }
            break;
        case CoordinateSystem::NED:
            switch (this->co_north)
            {
                case ArenaDirection::RIGHT:
                    std::cout << R"( 
     │                          │
     │   z  ⓧ → x               │
     │    y ↓                   │)";
                    break;
                case ArenaDirection::FAR_SIDE:
                    std::cout << R"( 
     │    x ↑                   │
     │   z  ⓧ → y               │
     │                          │)";
                    break;
                case ArenaDirection::LEFT:
                    std::cout << R"( 
     │    y ↑                   │
     │  x ← ⓧ z                 │
     │                          │)";
                    break;
                case ArenaDirection::NEAR_SIDE:
                    std::cout << R"( 
     │                          │
     │  y ← ⓧ z                 │
     │    x ↓                   │)";
                    break;
                case ArenaDirection::TRUE_NORTH: {
                    int rounded = static_cast<int>(std::round(this->true_north_deg));
                    char sign = (this->true_north_deg < 0) ? '-' : '+';
                    std::cout << R"( 
     │north )" << sign << std::setw(3) << std::setfill(' ') << std::abs(rounded) << R"(° from far side │)" << R"(
     │      ⓧ                   │
     │     z                    │)";
                    break;
                }
            }
            break;
        case CoordinateSystem::ENU:
            switch (this->co_north)
            {
                case ArenaDirection::FAR_SIDE:
                    std::cout << R"( 
     │    y ↑                   │
     │   z  ⊙ → x               │
     │                          │)";
                    break;
                case ArenaDirection::LEFT:
                    std::cout << R"( 
     │    x ↑                   │
     │  y ← ⊙ z                 │
     │                          │)";
                    break;
                case ArenaDirection::NEAR_SIDE:
                    std::cout << R"( 
     │                          │
     │  x ← ⊙ z                 │
     │    y ↓                   │)";
                    break;
                case ArenaDirection::RIGHT:
                    std::cout << R"( 
     │                          │
     │   z  ⊙ → y               │
     │    x ↓                   │)";
                    break;
                case ArenaDirection::TRUE_NORTH: {
                    int rounded = static_cast<int>(std::round(this->true_north_deg));
                    char sign = (this->true_north_deg < 0) ? '-' : '+';
                    std::cout << R"( 
     │north )" << sign << std::setw(3) << std::setfill(' ') << std::abs(rounded) << R"(° from far side │)" << R"(
     │      ⊙                   │
     │     z                    │)";
                    break;
                }
            }
            break;

    }
    
    std::cout<<R"(
     │                          │
     +──────────────────────────+
     │    Observers  (near)     │
     └──────────────────────────┘
     Assuming calibration triangle 
     long-edge has pointed to )" << this->long_edge << std::endl;
}
*/

// Function to simulate getch() in Linux
char getch(void) {
    struct termios oldt, newt;
    char ch;

    // Get the current terminal attributes
    tcgetattr(STDIN_FILENO, &oldt);
    
    // Set new terminal attributes for raw input
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO); // Disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    // Read a single character
    ch = getchar();

    // Restore old terminal attributes
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}
