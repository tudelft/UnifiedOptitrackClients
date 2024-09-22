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

#ifndef H_MOCAP
#define H_MOCAP

#include <vector>
#include <mutex>
#include <boost/program_options.hpp>

#include <iostream>
#include <csignal>
#include <thread>

#include "pose_calculations.hpp"
#include "agent.hpp"

constexpr unsigned int MAX_TRACKED_RB = 10;

class Mocap
{
private:
    // container for tracking the rigid bodies
    uint8_t nTrackedRB;
    int trackedRB[MAX_TRACKED_RB];
    bool unpublishedDataRB[MAX_TRACKED_RB];
    pose_t poseRB[MAX_TRACKED_RB];
    std::mutex poseMutexes[MAX_TRACKED_RB];
    pose_der_t poseDerRB[MAX_TRACKED_RB];
    std::mutex poseDerMutexes[MAX_TRACKED_RB];

    FilteredDifferentiator derFilter[MAX_TRACKED_RB];

    Agent* agent;

    //void publish_loop(void) {
    //    if (this->agent) {
    //        this->agent->publish_data();
    //    }

    //    return;
    //}

protected:
    double fSample;

    void processNewPose(int idx, pose_t& newPose) {
        // calculate derivative
        pose_der_t newPoseDer = derFilter[idx].apply(newPose);

        /* Thread safely setting the new values */
        // Lock respective mutex
        this->poseMutexes[idx].lock();
            memcpy(&(this->poseRB[idx]), &newPose, sizeof(pose_t));
        this->poseMutexes[idx].unlock();

        this->poseDerMutexes[idx].lock();
            memcpy(&(this->poseDerRB[idx]), &newPoseDer, sizeof(pose_der_t));
        this->poseDerMutexes[idx].unlock();

        // set unpublished
        this->unpublishedDataRB[idx] = true;

        // no throttling for now, TODO
        if (this->agent) {
            this->agent->print_data(idx, newPose, newPoseDer);
            this->agent->publish_data(idx, newPose, newPoseDer);
        }
    };

    int trackRB(unsigned int id) {
        int i = this->getIndexRB(id);
        if (i > -1) { return i; } // already tracked, that's fine
        if (this->nTrackedRB >= MAX_TRACKED_RB) { return -1; } // cannot add, too many RBs
        this->trackedRB[this->nTrackedRB] = id;
        return this->nTrackedRB++;
    };

    int getIndexRB(int id) { 
        for (unsigned int i=0; i < this->nTrackedRB; i++) {
            if (this->trackedRB[i] == id)
                return i;
        }
        return -1;
    };

public:
    Mocap();
    Mocap(const Mocap &other);
    ~Mocap();
    void enroll_agent(Agent *a) {
        this->agent = a;
    }

    virtual void banner();

    // Extra Program Options
    virtual void add_extra_po(boost::program_options::options_description &desc);
    virtual void parse_extra_po(const boost::program_options::variables_map &vm);
    virtual int connect();
    /* Returns the seconds since the a timestamp in
     * MoCap reference time */
    virtual double seconds_since_mocap_ts(uint64_t us);

    //unsigned int getStreamingId(unsigned int i) { return this->streaming_ids[i]; };
    //std::vector<unsigned int> getStreamingIds() { return this->streaming_ids; };
    int8_t getNTrackedRB()                      { return this->nTrackedRB; }
    int getIdRB(unsigned int idx)               { return this->trackedRB[idx]; }
    bool isUnpublishedRB(unsigned int idx)      { return this->unpublishedDataRB[idx]; }
    pose_t getPoseRB(unsigned int idx){
        // Lock respective mutex
        this->poseMutexes[idx].lock();
            pose_t pose(this->poseRB[idx]);
        this->poseMutexes[idx].unlock();
        return pose;
    };
    pose_der_t getPoseDerRB(unsigned int idx){
        // Lock respective mutex
        this->poseDerMutexes[idx].lock();
            pose_der_t pose_der(this->poseDerRB[idx]);
        this->poseDerMutexes[idx].unlock();
        return pose_der;
    };
    void setPublishedRB(unsigned int idx)
    {
        this->unpublishedDataRB[idx] = false;
    }
    void setPublishedAllRB(void)
    {
        for (unsigned int idx = 0; idx < getNTrackedRB(); idx++)
            this->unpublishedDataRB[idx] = false;
    }
};


#endif