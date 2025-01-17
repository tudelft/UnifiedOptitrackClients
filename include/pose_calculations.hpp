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

#ifndef H_POSE_CALCULATIONS
#define H_POSE_CALCULATIONS

#include <iostream>
#include <csignal>
#include <cstdint>
#include <cmath>

enum CoordinateSystem { NED, ENU };
enum ArenaDirection{ RIGHT=0, FAR_SIDE, LEFT, NEAR_SIDE, TRUE_NORTH };

typedef struct pose_s {
    uint64_t timeUs;
    float x;
    float y;
    float z;
    float qx;
    float qy;
    float qz;
    float qw;
} pose_t;

typedef struct twist_s {
    uint64_t timeUs;
    float vx;
    float vy;
    float vz;
    float wx;
    float wy;
    float wz;
} twist_t;

//pose_t transform_pose(const CoordinateSystem co,
//                      const ArenaDirection co_north,
//                      const float true_north_deg,
//                      const UpAxis up_axis,
//                      const ArenaDirection long_edge,
//                      const ArenaDirection craft_nose,
//                      const pose_t newPose);

typedef struct quaternions_s {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

typedef struct rotationMatrix_s {
    float m[3][3];
} rotationMatrix_t;

void quaternion_of_rotationMatrix(quaternion_t *q, const rotationMatrix_t *r);

class PoseDifferentiator
{
    protected:
        pose_t _pose;
        twist_t _unfiltered;
        bool _valid;
    public:
        PoseDifferentiator() : _valid{false} { _pose.qw = 1.; };
        //~PoseDifferentiator();
        twist_t getUnfiltered() { return _unfiltered; };
        virtual twist_t getFiltered() { return _unfiltered; };
        virtual twist_t apply(pose_t newPose) {
            // default: no filtering at all
            newSample(newPose);
            return _unfiltered;
        };

    protected:
        void newSample(pose_t newPose);
};

class FilteredPoseDifferentiator : public PoseDifferentiator
{
    private:
        float _fBreakVel;
        float _fBreakRate;
        float _fSample;
        float _kVel;
        float _kRate;
        twist_t _filtered;
        bool _initialized;

    public:
        FilteredPoseDifferentiator(float fBreakVel, float fBreakRate, float fSample);
        twist_t apply(pose_t newPose);
};

#endif // H_POSE_CALCULATINOS