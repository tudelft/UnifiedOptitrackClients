#ifndef H_CYBERZOO_OPTITRACK_CLIENT
#define H_CYBERZOO_OPTITRACK_CLIENT

#include <vector>
#include "NatNetClient.h"
#include "NatNetTypes.h"
#include "pose_calculations.hpp"

#include <iostream>

#define MAX_TRACKED_RB 10


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

    // container for tracking the rigid bodies
    uint8_t nTrackedRB;
    int trackedRB[MAX_TRACKED_RB];
    bool validRB[MAX_TRACKED_RB];
    pose_t poseRB[MAX_TRACKED_RB];
    pose_der_t poseDerRB[MAX_TRACKED_RB];
    pose_der_t poseDerRawRB[MAX_TRACKED_RB];
    bool printMessages;
    double fSample;
    sServerDescription serverConfig;

    FilteredDifferentiator derFilter[MAX_TRACKED_RB];

    void read_po(int argc, char const *argv[]);

    void print_startup() const;
    void print_coordinate_system() const;
    ErrorCode connectAndDetectServerSettings();


protected:
   

public:
    CyberZooMocapClient(int argc, char const *argv[]);
    ~CyberZooMocapClient();
    void natnet_data_handler(sFrameOfMocapData* data);
    int getIndexRB(int id) { 
        for (int i=0; i < this->nTrackedRB; i++) {
            if (this->trackedRB[i] == id)
                return i;
        }
        return -1;
    };
    int trackRB(int id) {
        int i = this->getIndexRB(id);
        if (i > -1) { return i; } // already tracked, that's fine
        if (this->nTrackedRB >= MAX_TRACKED_RB) { return -1; } // cannot add, too many RBs
        this->trackedRB[this->nTrackedRB] = id;
        return this->nTrackedRB++;
    };
    bool untrackRB(int id) {
        int i = this->getIndexRB(id);
        if (i == -1) { return true; } // already not tracked
        else { this->trackedRB[i] == -1; }
        this->nTrackedRB--;
        return true;
    };
    bool setPoseRB(int id, pose_t pose) {
        int i = this->getIndexRB(id);
        if (i == -1) { return false; } // not tracked; abort
        memcpy(&(this->poseRB[i]), &pose, sizeof(pose_t));
        return true;
    };
    bool setPoseDerRB(int id, pose_der_t poseDer) {
        int i = this->getIndexRB(id);
        if (i == -1) { return false; } // not tracked; abort
        memcpy(&(this->poseDerRB[i]), &poseDer, sizeof(pose_der_t));
        return true;
    };
};


#endif //H_CYBERZOO_OPTITRACK_CLIENT