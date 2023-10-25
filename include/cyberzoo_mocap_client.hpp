#ifndef H_CYBERZOO_OPTITRACK_CLIENT
#define H_CYBERZOO_OPTITRACK_CLIENT

#include <vector>
#include "NatNetClient.h"
#include "NatNetTypes.h"


enum CoordinateSystem { UNCHANGED=0, NED, ENU};
enum UpAxis { NOTDETECTED=-1, X=0, Y, Z };

class CyberZooMocapClient 
{
private:
    float publish_frequency;
    std::vector<unsigned int> streaming_ids;
    CoordinateSystem co;
    NatNetClient* pClient;
    sNatNetClientConnectParams connectParams;
    UpAxis upAxis;


    void read_po(int argc, char const *argv[]);

    void print_startup() const;
    void print_coordinate_system() const;
    ErrorCode attempt_server_connect();
    void natnet_data_handler(sFrameOfMocapData* data, void* pUserData) const;


protected:
   

public:
    CyberZooMocapClient(int argc, char const *argv[]);
    ~CyberZooMocapClient();
};

#endif //H_CYBERZOO_OPTITRACK_CLIENT