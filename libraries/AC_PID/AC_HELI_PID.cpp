/// @file	AC_HELI_PID.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_HELI_PID.h"

const AP_Param::GroupInfo AC_HELI_PID::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("P",    0, AC_HELI_PID, _kp, 0),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO("I",    1, AC_HELI_PID, _ki, 0),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO("D",    2, AC_HELI_PID, _kd, 0),

    // @Param: VFF
    // @DisplayName: Velocity FF FeedForward Gain
    // @Description: Velocity FF Gain which produces an output value that is proportional to the demanded input
    AP_GROUPINFO("VFF",    4, AC_HELI_PID, _ff, 0),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO("IMAX", 5, AC_HELI_PID, _imax, 0),

    // @Param: FILT
    // @DisplayName: PID Input filter frequency in Hz
    // @Description: PID Input filter frequency in Hz
    AP_GROUPINFO("FILT", 6, AC_HELI_PID, _filt_hz, AC_PID_FILT_HZ_DEFAULT),

    // @Param: ILMI
    // @DisplayName: I-term Leak Minimum
    // @Description: Point below which I-term will not leak down
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("ILMI", 7, AC_HELI_PID, _leak_min, AC_PID_LEAK_MIN),

    // @Param: NOTCH
    // @DisplayName: PID Input filter frequency in Hz
    // @Description: Input filter frequency in Hz
    // @Unit: Hz
    AP_GROUPINFO("NTCH", 8, AC_HELI_PID, _notch_hz, AC_PID_NOTCH_HZ_DEFAULT),


    // @Param: NOTCH_Q
    // @DisplayName: PID Notch filter Q factor
    // @Description: Notch filter Q factor changing depth and width
    // @Unit: none
    AP_GROUPINFO("NCHQ", 9, AC_HELI_PID, _notch_Q, AC_PID_NOTCH_Q_DEFAULT),
    // index 8 was for AFF, now removed

    AP_GROUPEND
};

/// Constructor for PID
AC_HELI_PID::AC_HELI_PID(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt, float initial_ff) :
    AC_PID(initial_p, initial_i, initial_d, initial_imax, initial_filt_hz, dt)
{
    _ff = initial_ff;
    _last_requested_rate = 0;
    _signal = 0.0f;
    _signal1 = 0.0f;
    _signal2 = 0.0f;
    _ntchsig = 0.0f;
    _ntchsig1 = 0.0f;
    _ntchsig2 = 0.0f;

    // reset input filter to first value received
    _flags._reset_notch = true;
}

// This is an integrator which tends to decay to zero naturally
// if the error is zero.

float AC_HELI_PID::get_leaky_i(float leak_rate)
{
    if(!is_zero(_ki) && !is_zero(_dt)){

        // integrator does not leak down below Leak Min
        if (_integrator > _leak_min){
            _integrator -= (float)(_integrator - _leak_min) * leak_rate;
        } else if (_integrator < -_leak_min) {
            _integrator -= (float)(_integrator + _leak_min) * leak_rate;
        }

        _integrator += ((float)_input * _ki) * _dt;
        if (_integrator < -_imax) {
            _integrator = -_imax;
            } else if (_integrator > _imax) {
            _integrator = _imax;
        }

        _pid_info.I = _integrator;
        return _integrator;
    }
    return 0;
}

// notch_filter_gyro - apply notch filter to gyro
float AC_HELI_PID::notch_filter_gyro(float input)
{
    // don't process inf or NaN
    if (!isfinite(input)) {
        return input;
    }

    // reset input filter to value received
    if (_flags._reset_notch) {
        _flags._reset_notch = false;
        _signal = input;
        _signal1 = input;
        _signal2 = input;
        _ntchsig = input;
        _ntchsig1 = input;
        _ntchsig2 = input;
    }

    _ntchsig=input;

// 2nd order notch filter
//    float notch_hz = 6.2f;
    if (_notch_hz > 0.0f ) {
      float notch_c = 1/tanf(M_PI*_dt*_notch_hz);
      float n0 = _notch_Q*(notch_c*notch_c+1.0f);
      float n1 =-2.0f*_notch_Q*(notch_c*notch_c-1.0f);
      float n2 = n0;
      float d0 = notch_c*notch_c*_notch_Q+notch_c+_notch_Q;
      float d1 = -2.0f*_notch_Q*(notch_c*notch_c-1.0f);
      float d2 = notch_c*notch_c*_notch_Q-notch_c+_notch_Q;
      _signal = (n0*_ntchsig+n1*_ntchsig1+n2*_ntchsig2-d1*_signal1-d2*_signal2)/d0;
    } else {
      _signal = _ntchsig;
      _signal1 = _ntchsig1;
      _signal2 = _ntchsig2;
    }

// Update n-1, and n-2 values of filter and signal for next iteration
    _ntchsig2 = _ntchsig1;
    _ntchsig1 = _ntchsig;
    _signal2 = _signal1;
    _signal1 = _signal;

return _signal;
}
