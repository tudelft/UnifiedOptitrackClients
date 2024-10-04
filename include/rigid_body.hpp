
#ifndef H_RIGID_BODY
#define H_RIGID_BODY

#include "pose_calculations.hpp"

class RigidBody
{
    private:
        pose_t poseENU;
        twist_t twistENU;
        ArenaDirection nose_direction;
        unsigned int unpublished_samples;

        //float velocity_filter_hz; // not implemented yet
        //float rates_filter_hz;
        //FilteredPoseDifferentiator poseDiff;
        PoseDifferentiator poseDiff;

    public:
        unsigned int id;

        RigidBody(unsigned int id, ArenaDirection nose_direction);

        pose_t getPoseIn(CoordinateSystem co);
        twist_t getTwistIn(CoordinateSystem co);
        void setNewPoseENU_NorthFarSide(pose_t& newPose);
        void setPublished() { this->unpublished_samples = 0; };
        unsigned int getNumUnpublishedSamples() { return this->unpublished_samples; };
};

#endif