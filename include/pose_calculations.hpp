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
enum ArenaDirection{RIGHT=0, FAR_SIDE, LEFT, NEAR_SIDE};

static pose_t transform_pose(const CoordinateSystem co,
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
            case ArenaDirection::FAR_SIDE:
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
            case ArenaDirection::NEAR_SIDE:
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

        float nose_rot_qw;
        float nose_rot_qx = 0.;
        float nose_rot_qy;
        float nose_rot_qz = 0.;

        switch(craft_nose) {
            case ArenaDirection::RIGHT:
                // left hand pi/2 rotation around Y axis (up)
                nose_rot_qw = -sqrt(2.0)/2.0;
                nose_rot_qy =  sqrt(2.0)/2.0;
                break;
            case ArenaDirection::FAR_SIDE:
                // no change, because this is what we have
                nose_rot_qw = 1.0;
                nose_rot_qy = 0.0;
                break;
            case ArenaDirection::LEFT:
                // right hand pi/2 rotation around Y axis (up)
                nose_rot_qw = sqrt(2.0)/2.0;
                nose_rot_qy = sqrt(2.0)/2.0;
                break;
            case ArenaDirection::NEAR_SIDE:
                // pi rotation around Y
                nose_rot_qw =  0.0;
                nose_rot_qy = -1.0;
                break;
        }

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
                result.x = z_copy;
                result.y = x_copy;
                result.z = y_copy;

                result.qx = qz_copy;
                result.qy = qx_copy;
                result.qz = qy_copy;
                break;
            case CoordinateSystem::NED:
                // Transform to NED
                result.x = x_copy;
                result.y = z_copy;
                result.z = -y_copy;

                result.qx = qx_copy;
                result.qy = qz_copy;
                result.qz = -qy_copy;
                break;
            default:
                break;
        }
    }

    return result;
}


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
        void newSample(pose_t newPose) {
            int64_t delta = newPose.timeUs - _pose.timeUs; // does this deal with overflows?

            if ((delta < 100) || (delta > 2e6) || (_pose.timeUs == 0)) {
                // likely uninitialized, or very old data
                _pose = newPose;
                _valid = false;
                return;
            }
            _valid = true;

            double iDelta = 1.0 / (static_cast<double>(delta) * 1e-6f);
            _unfiltered.timeUs = newPose.timeUs;
            _unfiltered.x = iDelta * (newPose.x - _pose.x);
            _unfiltered.y = iDelta * (newPose.y - _pose.y);
            _unfiltered.z = iDelta * (newPose.z - _pose.z);

            // https://mariogc.com/post/angular-velocity-quaternions/
            double q1[4] = {_pose.qw, _pose.qx, _pose.qy, _pose.qz};
            double q2[4] = {newPose.qw, newPose.qx, newPose.qy, newPose.qz};
            _unfiltered.wx = 2.*iDelta * (q1[0]*q2[1] - q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2]);
            _unfiltered.wy = 2.*iDelta * (q1[0]*q2[2] + q1[1]*q2[3] - q1[2]*q2[0] - q1[3]*q2[1]);
            _unfiltered.wz = 2.*iDelta * (q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] - q1[3]*q2[0]);

            _pose = newPose;
        };
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
        FilteredDifferentiator(double fBreakVel, double fBreakRate, double fSample) : PureDifferentiator(), _initialized{false}
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

        FilteredDifferentiator() : FilteredDifferentiator(1., 1., 1.) {
            // C++11
            // probably you can instead use a vector instead of a array to store FilteredDifferentiator instances?
        }

        pose_der_t apply(pose_t newPose) {
            newSample(newPose);
            if (!_valid)
                return _filtered;

            if (_initialized) {
                _filtered.x = _kVel * _filtered.x  +  (1.-_kVel) * _unfiltered.x; 
                _filtered.y = _kVel * _filtered.y  +  (1.-_kVel) * _unfiltered.y; 
                _filtered.z = _kVel * _filtered.z  +  (1.-_kVel) * _unfiltered.z; 
                _filtered.wx = _kRate * _filtered.wx  +  (1.-_kRate) * _unfiltered.wx;
                _filtered.wy = _kRate * _filtered.wy  +  (1.-_kRate) * _unfiltered.wy; 
                _filtered.wz = _kRate * _filtered.wz  +  (1.-_kRate) * _unfiltered.wz; 
            } else {
                _filtered = _unfiltered;
                _initialized = true;
            }

            return _filtered;
        };
};

#endif // H_POSE_CALCULATINOS