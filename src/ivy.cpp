#include "cyberzoo_mocap_client.hpp"
#include <glib.h>
#include <Ivy/ivy.h>
#include <Ivy/ivyglibloop.h>
#include <Ivy/ivyloop.h>
#include <pthread.h>
#include <unistd.h>

class NatNet2Ivy : public CyberZooMocapClient
{
public:
    NatNet2Ivy(int argc, char const *argv[], uint8_t ac_id) : CyberZooMocapClient(argc, argv)
    {
        _ac_id = ac_id;
        _freq = 100; // HZ
    }

    void publishLoop() {
        while (true) {
            usleep((unsigned int) (1000000.f / _freq)); // replace this by a scheduled thread somehow?
            // TODO: mutex lock
            unsigned int id = 126;

            if (!isValidRB(id)) { continue; }

            pose_t pose;
            pose_der_t poseDer;
            getPoseRB(id, &pose); // should not need to check return flags
            getPoseDerRB(id, &poseDer);
            IvySendMsg("datalink EXTERNAL_POSE %d %lu  %f %f %f  %f %f %f  %f %f %f %f",
                _ac_id, pose.timeUs/1000,  //todo: probably not the right timestamp
                pose.x, pose.y, pose.z,
                poseDer.x, poseDer.y, poseDer.z,
                pose.qw, pose.qx, pose.qy, pose.qz);

            // mutex unlock
        }
    };
private:
    uint8_t _ac_id;
    uint16_t _freq;
};

void *runPublisher(void* ptr) {
    NatNet2Ivy* that = (NatNet2Ivy *) ptr;
    that->publishLoop();
    return NULL;
}

void *keyListener(void* ptr) {
    NatNet2Ivy* that = (NatNet2Ivy *) ptr;
    that->listenToKeystrokes();
    return NULL;
}

int main(int argc, char const *argv[])
{
    uint8_t ac_id = 2;
    NatNet2Ivy client = NatNet2Ivy(argc, argv, ac_id);
    if (!client.isInitialized()) {
        return 1;
    }

    IvyInit ("NatNet2Ivy", "NatNet2Ivy READY", NULL, NULL, NULL, NULL);
    IvyStart("127.255.255.255");

    pthread_t pub;
    pthread_create( &pub, NULL, runPublisher, (void*) &client);

    pthread_t keys;
    pthread_create( &keys, NULL, keyListener, (void*) &client);

    IvyMainLoop();

    return 0;
}

