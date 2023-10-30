#ifndef H_POSE_CALCULATIONS
#define H_POSE_CALCULATIONS

#include <iostream>

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

            double iDelta = 1 / (static_cast<double>(delta) / 1e6f);
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
            _kVel = 1. / (1. + _fSample / (2. * 3.1415 * _fBreakVel) );
            _kRate = 1. / (1. + _fSample / (2. * 3.1415 * _fBreakRate) );
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