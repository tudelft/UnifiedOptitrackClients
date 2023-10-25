#include "cyberzoo_mocap_client.hpp"

#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

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

CyberZooMocapClient::CyberZooMocapClient(int argc, char const *argv[])
    : publish_frequency{100.0}, streaming_ids{1}, co{CoordinateSystem::UNCHANGED}, pClient{NULL}, upAxis{UpAxis::NOTDETECTED}
{
    this->print_startup();
    this->read_po(argc, argv);

    this->pClient = new NatNetClient();

    ErrorCode ret = this->attempt_server_connect();
    if (ret != ErrorCode_OK) {
        // returning from main is best for cleanup?
        return;
    }

    //this->pClient->SetFrameReceivedCallback( &(this->natnet_data_handler), this->pClient )

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

ErrorCode CyberZooMocapClient::attempt_server_connect()
{
    this->connectParams.connectionType = ConnectionType_Multicast;
    this->connectParams.serverCommandPort = NATNET_DEFAULT_PORT_COMMAND;
    this->connectParams.serverDataPort = NATNET_DEFAULT_PORT_DATA;
    this->connectParams.serverAddress = "192.168.0.153";
    //this->connectParams.localAddress = "192.168.0.255"; // better to leave these blank, then it autodetects, i think
    //this->connectParams.multicastAddress = "224.0.0.1";
    this->connectParams.subscribedDataOnly = false; // no idea what this does
    memset(this->connectParams.BitstreamVersion, 0, sizeof(this->connectParams.BitstreamVersion)); // no idea what this is

    ErrorCode ret = this->pClient->Connect(this->connectParams);
    std::cout << std::endl << "Attemption connect... ";
    if (ret == ErrorCode_OK)
        std::cout<<"Successful!"<<std::endl;
    else {
        std::cout<<"Failed with error code "<<ret<<std::endl;
        return ret;
    }

    // detect up axis
    void* response;
    int nBytes;
    std::cout<<"Detecting up axis... ";
    ret = this->pClient->SendMessageAndWait("GetProperty,,Up Axis", &response, &nBytes);
	if (ret == ErrorCode_OK) {
        this->upAxis = static_cast<UpAxis>( ((char*)response)[0] - '0' );
        std::cout << this->upAxis << std::endl;
	} else {
        std::cout << "Error code " << ret << std::endl;
        return ret;
    }

    return ErrorCode_OK;
}

void CyberZooMocapClient::natnet_data_handler(sFrameOfMocapData* data, void* pUserData) const
{
    return;
}
