#include "cyberzoo_mocap_client.hpp"

#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>
#include <functional>

#ifdef _WIN32
#   include <conio.h>
#else
#   include <unistd.h>
#   include <termios.h>
#endif

// helper function to get character presses
#ifndef _WIN32
char getch()
{
    char buf = 0;
    termios old = { 0 };

    fflush( stdout );

    if ( tcgetattr( 0, &old ) < 0 )
        perror( "tcsetattr()" );

    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;

    if ( tcsetattr( 0, TCSANOW, &old ) < 0 )
        perror( "tcsetattr ICANON" );

    if ( read( 0, &buf, 1 ) < 0 )
        perror( "read()" );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;

    if ( tcsetattr( 0, TCSADRAIN, &old ) < 0 )
        perror( "tcsetattr ~ICANON" );

    //printf( "%c\n", buf );

    return buf;
}
#endif

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <NatNetRequests.h>

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

std::ostream& operator<<(std::ostream& lhs, UpAxis e) {
    switch(e) {
    case NOTDETECTED: lhs << "NOTDETECTED"; break;
    case X: lhs << "X"; break;
    case Y: lhs << "Y"; break;
    case Z: lhs << "Z"; break;
    }
    return lhs;
}

void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData) {
    CyberZooMocapClient* that = (CyberZooMocapClient*) pUserData;
    that->natnet_data_handler(data);
};

CyberZooMocapClient::CyberZooMocapClient(int argc, char const *argv[])
    : publish_frequency{100.0}, streaming_ids{1}, co{CoordinateSystem::UNCHANGED}, pClient{NULL}, upAxis{UpAxis::NOTDETECTED}, printMessages{false},
    nTrackedRB{0}, validRB{false}
{

    // TODO: use builtin forward prediction with the latency estimates plus a 
    // user-defined interval (on the order of 10ms)?

    this->print_startup();
    this->read_po(argc, argv);

    this->pClient = new NatNetClient();

    ErrorCode ret = this->connectAndDetectServerSettings();
    if (ret != ErrorCode_OK) {
        // returning from main is best for cleanup?
        return;
    }

    for (int i=0; i < MAX_TRACKED_RB; i++)
        derFilter[i] = FilteredDifferentiator(10., 10., this->fSample);

    for (unsigned int i : this->streaming_ids) {
        int idx = this->trackRB(i);
        if (idx == -1) {
            std::cout << "Cannot track Rigid Body with id " << i << ". Too many rigid bodies" << std::endl;
        }
    }

    this->pClient->SetFrameReceivedCallback( DataHandler, this );

    std::cout << std::endl << "Listening to messages! Press q to quit, Press t to toggle message printing" << std::endl;
	while ( const int c = getch() )
    {
        switch(c)
        {
            case 'q':
                return;
            case 't':
                this->printMessages ^= true; // toggle
                break;
        }
    }
}

CyberZooMocapClient::~CyberZooMocapClient()
{
}


void CyberZooMocapClient::read_po(int argc, char const *argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("publish_frequency,f", po::value<float>(), "publishing frequency of the MoCap odometry")
        ("streaming_ids,s", po::value<std::vector<unsigned int> >()->multitoken(), "streaming ids to track")
        ("coordinate_system,c", po::value<std::string>(), "coordinate system convention to use [unchanged, ned, enu]")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);    

    if (vm.count("help")) {
        std::cout << desc << "\n";
        exit(1);
    }

    if (vm.count("publish_frequency")) {
        this->publish_frequency = vm["compression"].as<float>();
        std::cout << "Publish frequency was set to " 
                  << this->publish_frequency << std::endl;
    } else {
        std::cout << "Publish frequency not set, defaulting to " 
                  << this->publish_frequency
                  << " Hz" << std::endl;   
    }

    if(vm.count("streaming_ids"))
    {
        this->streaming_ids = vm["streaming_ids"].as<std::vector<unsigned int>>();
        std::cout << "Streaming IDs set to";
        for(unsigned int id : this->streaming_ids) std::cout << " " << id << " ";
        std::cout << std::endl;
    }
    else
    {
        std::cout << "Streaming IDs not set, defaulting to";
        for(unsigned int id : this->streaming_ids) std::cout << " " << id << " ";
        std::cout << std::endl;
    }

    if(vm.count("coordinate_system"))
    {
        std::string co_name = vm["coordinate_system"].as<std::string>();
        boost::algorithm::to_lower(co_name);

        if(co_name.compare("unchanged") == 0)
        {
            this->co = CoordinateSystem::UNCHANGED;
        }
        else if(co_name.compare("ned") == 0)
        {
            this->co = CoordinateSystem::NED;
        }else if (co_name.compare("enu") == 0)
        {
            this->co = CoordinateSystem::ENU;
        }
        else
        {
            std::cout << "Coordinate system " << co_name << " not definied. Exiting" << std::endl;
            exit(0);
        }
        
        std::cout << "Coordinate system set to " << this->co << std::endl;

    }
    else
    {
        std::cout << "Coordinate System not set, defaulting to "
                  << this->co << std::endl;
    }
}

void CyberZooMocapClient::print_startup() const
{
    std::cout<< R"(
    ####################################################
    ##      _____     __          ____  ____  ____    ## 
    ##     / ___/_ __/ /  ___ ___/_  / / __ \/ __ \   ##
    ##    / /__/ // / _ \/ -_) __// /_/ /_/ / /_/ /   ##
    ##    \___/\_, /_.__/\__/_/  /___/\____/\____/    ##
    ##        /___/                                   ##
    ##                                                ##
    ####################################################
    )" << '\n';
}

void CyberZooMocapClient::print_coordinate_system() const
{
    std::cout<< R"( 
               far
     +──────────────────────────+
     │                          │
     │        ⊙ ⊙               │
     │       ⊙ + ⊙              │
     │        ⊙ ⊙               │
left │                          │ right
     │                          │
     │                          │
     │                          │
     +──────────────────────────+
     │    Observers  (near)     │
     └──────────────────────────┘
    )" << '\n';
}

ErrorCode CyberZooMocapClient::connectAndDetectServerSettings()
{
    this->connectParams.connectionType = ConnectionType_Multicast;
    this->connectParams.serverCommandPort = NATNET_DEFAULT_PORT_COMMAND;
    this->connectParams.serverDataPort = NATNET_DEFAULT_PORT_DATA;
    this->connectParams.serverAddress = "192.168.209.81";
    //this->connectParams.localAddress = "192.168.0.255"; // better to leave these blank, then it autodetects, i think
    this->connectParams.multicastAddress = "224.0.0.1";
    this->connectParams.subscribedDataOnly = false; // no idea what this does
    memset(this->connectParams.BitstreamVersion, 0, sizeof(this->connectParams.BitstreamVersion)); // no idea what this is

    ErrorCode ret = this->pClient->Connect(this->connectParams);
    std::cout << std::endl << "Attemption connect... ";
    if (ret == ErrorCode_OK)
        std::cout<<"Successful!"<<std::endl;
    else {
        std::cout<<"Failed with error code "<<ret<<std::endl;
        std::cout<<std::endl<<"Verify the following: " << std::endl;
        std::cout<<"1. Connected to Motive network" << std::endl;
        std::cout<<"2. Interface is NOT set to 'local' in Motive 'Data Streaming Pane'" << std::endl;
        return ret;
    }

    // check for version >2
    bool verAtLeast2 = true;
    bool forceVerAtLeast2 = false;
    if (!verAtLeast2 && forceVerAtLeast2) {
        return ErrorCode_Other;
    }

    void* response;
    int nBytes;

    // detect frame rate
    std::cout<<"Detecting frame rate... ";
    ret = this->pClient->SendMessageAndWait("FrameRate", &response, &nBytes);
    if (ret == ErrorCode_OK) {
        this->fSample = (double) *((float*)response);
        std::cout << this->fSample << "Hz" << std::endl;
    } else {
        std::cout << "Error code " << ret << std::endl;
        return ret;
    }

    // detect host clock settings for accurate time calcs
    std::cout<<"Detecting Server Configuration.. ";
    memset( &(this->serverConfig), 0, sizeof( this->serverConfig ) );
    ret = this->pClient->GetServerDescription(&(this->serverConfig));
    if (ret == ErrorCode_OK) {
        std::cout << "Done" << std::endl;
    } else {
        std::cout << "Error code " << ret << std::endl;
        return ret;
    }

    // detect up axis
    if (verAtLeast2) {
        std::cout<<"Detecting up axis... ";
        ret = this->pClient->SendMessageAndWait("GetProperty,,Up Axis", &response, &nBytes);
        if (ret == ErrorCode_OK) {
            this->upAxis = static_cast<UpAxis>( ((char*)response)[0] - '0' );
            std::cout << this->upAxis << std::endl;
        } else {
            std::cout << "Error code " << ret << std::endl;
            return ret;
        }
    } else {
        this->upAxis = UpAxis::Y;
        std::cout << "WARNING: Motive version less than 2. Assuming y_up is set." << std::endl;
    }

    // inform user
    std::cout << "INFO: if you see this message but you still don't receive messages, check:" << std::endl;
    std::cout << "1. Rigid body id(s) are correct" << std::endl;
    std::cout << "2. Rigid body(s) are selected in Motive" << std::endl;
    std::cout << "3. 'Multicast' is selected in 'Data Streaming Pane' in Motive" << std::endl;

    return ErrorCode_OK;
}

void CyberZooMocapClient::natnet_data_handler(sFrameOfMocapData* data)
{

    uint64_t timeAtExpoUs = data->CameraMidExposureTimestamp / (this->serverConfig.HighResClockFrequency / 1e6);

    if (this->printMessages)
        std::cout << "Received data for " << data->nRigidBodies << " rigid bodies for host time: " << timeAtExpoUs << "us" << std::endl;

	for(int i=0; i < data->nRigidBodies; i++) {
        int idx = this->getIndexRB(data->RigidBodies[i].ID);
        if (idx == -1)
            continue; // untracked by us

        bool bTrackingValid = data->RigidBodies[i].params & 0x01;
        if (bTrackingValid) {
            validRB[idx] = true;
        } else {
            validRB[idx] = false;
            continue;
        }

        pose_t newPose {
            .timeUs = timeAtExpoUs,
            .x = data->RigidBodies[i].x,
            .y = data->RigidBodies[i].y,
            .z = data->RigidBodies[i].z,
            .qx = data->RigidBodies[i].qx,
            .qy = data->RigidBodies[i].qy,
            .qz = data->RigidBodies[i].qz,
            .qw = data->RigidBodies[i].qw,
        };

        poseDerRB[idx] = derFilter[idx].apply(newPose);
        poseDerRawRB[idx] = derFilter[idx].getUnfiltered();
        poseRB[idx] = newPose;

        if (this->printMessages) {
		    printf("Rigid Body [ID=%d Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
		    printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
		    printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
		    	data->RigidBodies[i].x,
		    	data->RigidBodies[i].y,
		    	data->RigidBodies[i].z,
		    	data->RigidBodies[i].qx,
		    	data->RigidBodies[i].qy,
		    	data->RigidBodies[i].qz,
		    	data->RigidBodies[i].qw);
            printf("\tvx\tvy\tvz\twx\twy\twz\n");
		    printf("\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\t%4.2f\n",
                poseDerRB[idx].x,
                poseDerRB[idx].y,
                poseDerRB[idx].z,
                poseDerRB[idx].wx,
                poseDerRB[idx].wy,
                poseDerRB[idx].wz);
        }
    }

    return;
}

