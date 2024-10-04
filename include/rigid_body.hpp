
#ifndef H_RIGID_BODY
#define H_RIGID_BODY

#include "pose_calculations.hpp"

class RigidBody
{
    private:
        pose_t poseENU;
        twist_t twistENU;
        ArenaDirection nose_direction;
        bool isPublished;

        //float velocity_filter_hz; // not implemented yet
        //float rates_filter_hz;
        //FilteredPoseDifferentiator poseDiff;
        PoseDifferentiator poseDiff;

    public:
        RigidBody();
        RigidBody(ArenaDirection nose_direction);

        pose_t getPoseIn(CoordinateSystem co);
        twist_t getTwistIn(CoordinateSystem co);
        void setDataENU(pose_t& newPos);
        void setPublished() { this->isPublished = true; };
        bool isPublished() { return this->isPublished; }
}

#endif