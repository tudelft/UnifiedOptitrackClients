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
#include <cmath>

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

typedef struct pose_der_s {
    uint64_t timeUs;
    float x;
    float y;
    float z;
    float wx;
    float wy;
    float wz;
} pose_der_t;

enum CoordinateSystem { UNCHANGED=0, NED, ENU};
enum UpAxis { NOTDETECTED=-1, X=0, Y, Z };
enum ArenaDirection{RIGHT=0, FAR_SIDE, LEFT, NEAR_SIDE, TRUE_NORTH};

pose_t transform_pose(const CoordinateSystem co,
                      const ArenaDirection co_north,
                      const float true_north_deg,
                      const UpAxis up_axis,
                      const ArenaDirection long_edge,
                      const ArenaDirection craft_nose,
                      const pose_t newPose);

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

class PureDifferentiator
{
    protected:
        pose_t _pose;
        pose_der_t _unfiltered;
        bool _valid;
    public:
        PureDifferentiator() : _valid{false} { _pose.qw = 1.; };
        //~PureDifferentiator();
        pose_der_t getUnfiltered() { return _unfiltered; };
        virtual pose_der_t getFiltered() { return _unfiltered; };
        virtual pose_der_t apply(pose_t newPose) {
            // default: no filtering at all
            newSample(newPose);
            return _unfiltered;
        };

    protected:
        void newSample(pose_t newPose);
};

class FilteredDifferentiator : public PureDifferentiator
{
    private:
        double _fBreakVel;
        double _fBreakRate;
        double _fSample;
        double _kVel;
        double _kRate;
        pose_der_t _filtered;
        bool _initialized;

    public:
        FilteredDifferentiator(double fBreakVel, double fBreakRate, double fSample);

        FilteredDifferentiator() : FilteredDifferentiator(1., 1., 1.) {
            // C++11
            // probably you can instead use a vector instead of a array to store FilteredDifferentiator instances?
        }

        pose_der_t apply(pose_t newPose);
};

#endif // H_POSE_CALCULATINOS