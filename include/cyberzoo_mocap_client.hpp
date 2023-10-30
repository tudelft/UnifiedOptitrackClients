#ifndef H_CYBERZOO_OPTITRACK_CLIENT
#define H_CYBERZOO_OPTITRACK_CLIENT

#include <vector>
#include <mutex>
#include "NatNetClient.h"
#include "NatNetTypes.h"
#include "pose_calculations.hpp"

#include <iostream>

#ifndef _WIN32
char getch();
#endif

constexpr unsigned int MAX_TRACKED_RB = 10;


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
    bool printMessages;
    uint8_t nTrackedRB;
    int trackedRB[MAX_TRACKED_RB];
    bool validRB[MAX_TRACKED_RB];
    pose_t poseRB[MAX_TRACKED_RB];
    std::mutex poseMutexes[MAX_TRACKED_RB];
    pose_der_t poseDerRB[MAX_TRACKED_RB];
    std::mutex poseDerMutexes[MAX_TRACKED_RB];
    double fSample;
    sServerDescription serverConfig;

    FilteredDifferentiator derFilter[MAX_TRACKED_RB];

    void read_po(int argc, char const *argv[]);
    void print_startup() const;
    void print_coordinate_system() const;
    ErrorCode connectAndDetectServerSettings();


protected:
   int trackRB(unsigned int id) {
        int i = this->getIndexRB(id);
        if (i > -1) { return i; } // already tracked, that's fine
        if (this->nTrackedRB >= MAX_TRACKED_RB) { return -1; } // cannot add, too many RBs
        this->trackedRB[this->nTrackedRB] = id;
        return this->nTrackedRB++;
    };
    bool untrackRB(unsigned int id) {
        int i = this->getIndexRB(id);
        if (i == -1) { return true; } // already not tracked
        else { this->trackedRB[i] = -1; }
        this->nTrackedRB--;
        return true;
    };

    /* Thread safe mutators and accessors for the current pose 
     * and pose derivative values. */
    bool setPoseRB(unsigned int id, pose_t pose) {
        int i = this->getIndexRB(id);
        if (i == -1) { return false; } // not tracked; abort
        // Lock respective mutex
        this->poseMutexes[i].lock();
        memcpy(&(this->poseRB[i]), &pose, sizeof(pose_t));
        this->poseMutexes[i].unlock();
        return true;
    };
    pose_t getPoseRB(unsigned int id){
        int i = this->getIndexRB(id);
        if (i == -1) { return pose_t(); } // not tracked; return empty pose;
        // Lock respective mutex
        this->poseMutexes[i].lock();
        pose_t pose = this->poseRB[i];
        this->poseMutexes[i].unlock();
        return pose;
    };
    bool setPoseDerRB(int id, pose_der_t poseDer) {
        int i = this->getIndexRB(id);
        if (i == -1) { return false; } // not tracked; abort
        // Lock respective mutex
        this->poseDerMutexes[i].lock();
        memcpy(&(this->poseDerRB[i]), &poseDer, sizeof(pose_der_t));
        this->poseDerMutexes[i].unlock();
        return true;
    };
    pose_der_t getPoseDerRB(unsigned int id){
        int i = this->getIndexRB(id);
        if (i == -1) { return pose_der_t(); } // not tracked; return empty pose;
        // Lock respective mutex
        this->poseDerMutexes[i].lock();
        pose_der_t pose_der = this->poseDerRB[i];
        this->poseMutexes[i].unlock();
        return pose_der;
    };

public:
    CyberZooMocapClient(int argc, char const *argv[]);
    CyberZooMocapClient(const CyberZooMocapClient &other);
    ~CyberZooMocapClient();
    void natnet_data_handler(sFrameOfMocapData* data);
    int getIndexRB(int id) { 
        for (unsigned int i=0; i < this->nTrackedRB; i++) {
            if (this->trackedRB[i] == id)
                return i;
        }
        return -1;
    };
    
};


#endif //H_CYBERZOO_OPTITRACK_CLIENT