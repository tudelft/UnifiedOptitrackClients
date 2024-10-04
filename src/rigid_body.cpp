

#include "rigid_body.hpp"
#include "pose_calculation.hpp"

//RigidBody::RigidBody( float fSample ) : RigidBody( fSample, 0.0f ) {
RigidBody::RigidBody() : RigidBody( ArenaDirection::FAR_SIDE ) {
}

RigidBody::RigidBody( ArenaDirection nose_direction) : 
    //velocity_filter_hz{20.f},
    //rates_filter_hz{10.f},
    isPublished{true},
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
    
}

twist_t RigidBody::getTwistIn(CoordinateSystem co)
{

}

void setDataENU(pose_t& newPos)
{
    this->poseENU = newPose;
    this->twistENU = this->poseDiff.apply(this->poseENU);
    this->isPublished = false;
}
