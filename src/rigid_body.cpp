

#include "rigid_body.hpp"
#include "pose_calculations.hpp"

RigidBody::RigidBody( unsigned int id, ArenaDirection nose_direction) : 
    //velocity_filter_hz{20.f},
    //rates_filter_hz{10.f},
    id{id},
    unpublished_samples{0}
{
    this->nose_direction = nose_direction;
    this->poseDiff = PoseDifferentiator(); 
    //this->poseDiff = FilteredPoseDifferentiator( 
    //    this->velocity_filter_hz,
    //    this->rates_filter_hz,
    //    fSample,
    //);
}

pose_t RigidBody::getPoseIn(CoordinateSystem co)
{
    // todo: convert
    return this->poseENU;    
}

twist_t RigidBody::getTwistIn(CoordinateSystem co)
{
    // todo: convert
    return this->twistENU;
}

void RigidBody::setNewPoseENU_NorthFarSide(pose_t& newPose)
{
    this->poseENU = newPose;
    this->twistENU = this->poseDiff.apply(this->poseENU);
    this->unpublished_samples++;
}
