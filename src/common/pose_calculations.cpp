// Copyright 2024 Anton Bredenbeck, Till Blaha (Delft University of Technology)
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

#include "pose_calculations.hpp"

std::ostream& operator<<(std::ostream& lhs, CoordinateSystem e) {
    switch(e) {
    //case UNCHANGED: lhs << "UNCHANGED"; break;
    case NED: lhs << "NED"; break;
    case ENU: lhs << "ENU"; break;
    }
    return lhs;
} 

std::ostream& operator<<(std::ostream& lhs, ArenaDirection e) {
    switch(e) {
    case RIGHT: lhs << "RIGHT"; break;
    case FAR: lhs << "FAR"; break;
    case LEFT: lhs << "LEFT"; break;
    case NEAR: lhs << "NEAR"; break;
    }
    return lhs;
} 

/*
pose_t transform_pose(const CoordinateSystem co, 
                      const ArenaDirection co_north,
                      const float true_north_deg,
                      const UpAxis up_axis,
                      const ArenaDirection long_edge,
                      const ArenaDirection craft_nose,
                      const pose_t newPose)
{
    pose_t result(newPose);
    
    if(co != CoordinateSystem::UNCHANGED)
    {

        float x_copy = result.x;
        float y_copy = result.y;
        float z_copy = result.z;

        float qw_copy = result.qw;
        float qx_copy = result.qx;
        float qy_copy = result.qy;
        float qz_copy = result.qz;

        switch(up_axis)
        {
            case UpAxis::X:
                // Transform from X-Up to Y-Up
                result.x = -y_copy;
                result.y = x_copy;
                result.z = z_copy;

                result.qw = qw_copy;
                result.qx = -qy_copy;
                result.qy = qx_copy;
                result.qz = qz_copy;
                break;
            case UpAxis::Y:
                // We do nothing because this is what we want to have
                break;
            case UpAxis::Z:
                // Transform from X-Up to Y-Up
                result.x = x_copy;
                result.y = z_copy;
                result.z = -y_copy;

                result.qw = qw_copy;
                result.qx = qx_copy;
                result.qy = qz_copy;
                result.qz = -qy_copy;
                break;
            case UpAxis::NOTDETECTED:
                // The up axis is not known. Abort
                std::cerr << "The up-axis is not detected. Aborting." << std::endl;
                std::raise(SIGINT);
                break;    
            default:
                break;
        }

        x_copy = result.x;
        y_copy = result.y;
        z_copy = result.z;

        qw_copy = result.qw;
        qx_copy = result.qx;
        qy_copy = result.qy;
        qz_copy = result.qz;

        switch(long_edge)
        {

            case ArenaDirection::RIGHT:
                // We do nothing because this is what we want to have
                break;
            case ArenaDirection::FAR:
                // Rotate to align in the yaw plane
                result.x = z_copy;
                result.z = -x_copy;

                result.qx = qz_copy;
                result.qz = -qx_copy;
                break;
            case ArenaDirection::LEFT:
                // Rotate to align in the yaw plane
                result.x = -x_copy;
                result.z = -z_copy;

                result.qx = -qx_copy;
                result.qz = -qz_copy;
                break;
            case ArenaDirection::NEAR:
                // Rotate to align in the yaw plane
                result.x = -z_copy;
                result.z = x_copy;

                result.qx = -qz_copy;
                result.qz = qx_copy;
                break;
        }

        qw_copy = result.qw;
        qx_copy = result.qx;
        qy_copy = result.qy;
        qz_copy = result.qz;

        float nose_rot_angle;
        switch(craft_nose)
        {
            // left hand pi/2 rotation around Y axis (up)
            case ArenaDirection::RIGHT: { nose_rot_angle = -M_PI/2.0; break; }
            // no change, because this is what we have
            case ArenaDirection::FAR: { nose_rot_angle = 0.0; break; }
            // right hand pi/2 rotation around Y axis (up)
            case ArenaDirection::LEFT: { nose_rot_angle = M_PI/2.0; break; }
            // pi rotation around Y
            case ArenaDirection::NEAR: { nose_rot_angle = M_PI; break; }
        }

        float co_north_angle;
        switch(co_north)
        {
            case ArenaDirection::RIGHT: { co_north_angle = -M_PI/2.0; break; }
            case ArenaDirection::FAR: { co_north_angle = 0.0; break; }
            case ArenaDirection::LEFT: { co_north_angle = M_PI/2.0; break; }
            case ArenaDirection::NEAR: { co_north_angle = M_PI; break; }
            case ArenaDirection::TRUE_NORTH:
                co_north_angle = true_north_deg * M_PI / 180.0;
                if (co == CoordinateSystem::NED)
                    co_north_angle *= -1.0;
                break;
        }

        float nose_rot_qw = cos((nose_rot_angle - co_north_angle)/2);
        float nose_rot_qx = 0.;
        float nose_rot_qy = sin((nose_rot_angle - co_north_angle)/2);
        float nose_rot_qz = 0.;

        // perform nose rotation as quaternion rotation https://gegcalculators.com/quaternion-multiplication-calculator-online/
        // result = q_copy * nose_rot  --> use some library here? does boost have quats?
        result.qw = qw_copy * nose_rot_qw - qx_copy * nose_rot_qx -  qy_copy * nose_rot_qy - qz_copy * nose_rot_qz;
        result.qx = qw_copy * nose_rot_qx + qx_copy * nose_rot_qw +  qy_copy * nose_rot_qz - qz_copy * nose_rot_qy;
        result.qy = qw_copy * nose_rot_qy - qx_copy * nose_rot_qz +  qy_copy * nose_rot_qw + qz_copy * nose_rot_qx;
        result.qz = qw_copy * nose_rot_qz + qx_copy * nose_rot_qy -  qy_copy * nose_rot_qx + qz_copy * nose_rot_qw;

        x_copy = result.x;
        y_copy = result.y;
        z_copy = result.z;

        qw_copy = result.qw;
        qx_copy = result.qx;
        qy_copy = result.qy;
        qz_copy = result.qz;

        switch(co)
        {
            case CoordinateSystem::ENU:
                // Transform to ENU
                result.x = +cos(co_north_angle) * z_copy + sin(co_north_angle) * x_copy;
                result.y = -sin(co_north_angle) * z_copy + cos(co_north_angle) * x_copy;
                result.z = y_copy;

                result.qx = +cos(co_north_angle) * qz_copy + sin(co_north_angle) * qx_copy;
                result.qy = -sin(co_north_angle) * qz_copy + cos(co_north_angle) * qx_copy;
                result.qz = qy_copy;
                break;
            case CoordinateSystem::NED:
                // Transform to NED
                result.x = +cos(co_north_angle) * x_copy - sin(co_north_angle) * z_copy;
                result.y = +sin(co_north_angle) * x_copy + cos(co_north_angle) * z_copy;
                result.z = -y_copy;

                result.qx = +cos(co_north_angle) * qx_copy - sin(co_north_angle) * qz_copy;
                result.qy = +sin(co_north_angle) * qx_copy + cos(co_north_angle) * qz_copy;
                result.qz = -qy_copy;
                break;
            default:
                break;
        }
    }

    return result;
}
*/

void quaternion_of_rotationMatrix(quaternion_t *q, const rotationMatrix_t *r) {
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    float trace = r->m[0][0] + r->m[1][1] + r->m[2][2];
    float s, si;
    if (trace > 1e-6f) {
        s = 0.5f * sqrtf( 1.0f + trace );
        si = 0.25f / s;
        q->w = s;
        q->x = si * ( r->m[2][1] - r->m[1][2] );
        q->y = si * ( r->m[0][2] - r->m[2][0] );
        q->z = si * ( r->m[1][0] - r->m[0][1] );
    } else {
        if ( r->m[0][0] > r->m[1][1] && r->m[0][0] > r->m[2][2] ) {
            s = 0.5f * sqrtf( 1.0f + r->m[0][0] - r->m[1][1] - r->m[2][2]);
            si = 0.25f / s;
            q->w = si * (r->m[2][1] - r->m[1][2] );
            q->x = s;
            q->y = si * (r->m[0][1] + r->m[1][0] );
            q->z = si * (r->m[0][2] + r->m[2][0] );
        } else if (r->m[1][1] > r->m[2][2]) {
            s = 0.5f * sqrtf( 1.0f + r->m[1][1] - r->m[0][0] - r->m[2][2]);
            si = 0.25f / s;
            q->w = si * (r->m[0][2] - r->m[2][0] );
            q->x = si * (r->m[0][1] + r->m[1][0] );
            q->y = s;
            q->z = si * (r->m[1][2] + r->m[2][1] );
        } else {
            s = 0.5f * sqrtf( 1.0f + r->m[2][2] - r->m[0][0] - r->m[1][1] );
            si = 0.25f / s;
            q->w = si * (r->m[1][0] - r->m[0][1] );
            q->x = si * (r->m[0][2] + r->m[2][0] );
            q->y = si * (r->m[1][2] + r->m[2][1] );
            q->z = s;
        }
    }
}

void PoseDifferentiator::newSample(pose_t newPose)
{
    int64_t delta = newPose.timeUs - _pose.timeUs; // does this deal with overflows?

    if ((delta < 100) || (delta > 2e6) || (_pose.timeUs == 0)) {
        // likely uninitialized, or very old data
        _pose = newPose;
        _valid = false;
        return;
    }
    _valid = true;

    float iDelta = 1.0 / (static_cast<float>(delta) * 1e-6f);
    _unfiltered.timeUs = newPose.timeUs;
    _unfiltered.vx = iDelta * (newPose.x - _pose.x);
    _unfiltered.vy = iDelta * (newPose.y - _pose.y);
    _unfiltered.vz = iDelta * (newPose.z - _pose.z);

    // https://mariogc.com/post/angular-velocity-quaternions/
    float q1[4] = {_pose.qw, _pose.qx, _pose.qy, _pose.qz};
    float q2[4] = {newPose.qw, newPose.qx, newPose.qy, newPose.qz};
    _unfiltered.wx = 2.*iDelta * (q1[0]*q2[1] - q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2]);
    _unfiltered.wy = 2.*iDelta * (q1[0]*q2[2] + q1[1]*q2[3] - q1[2]*q2[0] - q1[3]*q2[1]);
    _unfiltered.wz = 2.*iDelta * (q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] - q1[3]*q2[0]);

    _pose = newPose;
}

FilteredPoseDifferentiator::FilteredPoseDifferentiator(float fBreakVel,
                                                       float fBreakRate,
                                                       float fSample)
    : PoseDifferentiator(), _initialized{false}, _filtered{0,
                                                           0, 0, 0,
                                                           0, 0, 0}
{
    _fSample = fSample;
    _fBreakVel = fBreakVel;
    _fBreakRate = fBreakRate;
    if (fBreakVel < 1e-3) {
        _kVel = 0.;
        std::cout << "WARNING: Velocity lowpass filter cannot be realized and will be disabled. Check fBreakVel." << std::endl;
        return;
    }
    if (fBreakRate < 1e-3) {
        _kVel = 0.;
        std::cout << "WARNING: Angular rate lowpass filter cannot be realized and will be disabled. Check fBreakRate." << std::endl;
        return;
    }

    // https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter
    _kVel = 1. / (1. + _fSample / (2. * M_PI * _fBreakVel) );
    _kRate = 1. / (1. + _fSample / (2. * M_PI * _fBreakRate) );
}

twist_t FilteredPoseDifferentiator::apply(pose_t newPose)
{
    newSample(newPose);
    if (!_valid)
        return _filtered;

    if (_initialized) {
        _filtered.vx = _kVel * _filtered.vx  +  (1.-_kVel) * _unfiltered.vx; 
        _filtered.vy = _kVel * _filtered.vy  +  (1.-_kVel) * _unfiltered.vy; 
        _filtered.vz = _kVel * _filtered.vz  +  (1.-_kVel) * _unfiltered.vz; 
        _filtered.wx = _kRate * _filtered.wx  +  (1.-_kRate) * _unfiltered.wx;
        _filtered.wy = _kRate * _filtered.wy  +  (1.-_kRate) * _unfiltered.wy; 
        _filtered.wz = _kRate * _filtered.wz  +  (1.-_kRate) * _unfiltered.wz; 
    } else {
        _filtered = _unfiltered;
        _initialized = true;
    }

    return _filtered;
}