#pragma once

/// @file	AC_HELI_PID.h
/// @brief	Helicopter Specific Rate PID algorithm, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <stdlib.h>
#include <cmath>
#include "AC_PID.h"

#define AC_PID_LEAK_MIN          0.1  // Default I-term Leak Minimum
#define AC_PID_NOTCH_HZ_DEFAULT  0.0f   // default notch input filter frequency
#define AC_PID_NOTCH_Q_DEFAULT  0.8f   // default notch input Q factor

/// @class	AC_HELI_PID
/// @brief	Heli PID control class
class AC_HELI_PID : public AC_PID {
public:

    /// Constructor for PID
    AC_HELI_PID(float initial_p, float initial_i, float initial_d, float initial_imax, float initial_filt_hz, float dt, float initial_ff);

    /// get_leaky_i - replacement for get_i but output is leaded at leak_rate
    float       get_leaky_i(float leak_rate);

    // notch_filter_gyro - notch filter gyro before PID
    //  gyro is notch filtered before the PID controllers are run
    //  this should be called before set_input_filter_all
    float       notch_filter_gyro(float input);

    // reset_filter - input filter will be reset to the next value provided to set_input()
    void        reset_notch();

    static const struct AP_Param::GroupInfo        var_info[];

private:
    // parameters
    AP_Float        _leak_min;
    AP_Float        _notch_hz;                   // PID Input notch filter frequency in Hz
    AP_Float        _notch_Q;                   // PID Input notch filter Q factor

    // flags
    struct ac_heli_pid_flags {
        bool        _reset_notch : 1;    // true when notch filter should be reset during next call to notch_filter_gyro
    } _flags;

    // internal variables
    float           _signal;                // current value of filter signal
    float           _signal1;               // n-1 value of filter signal
    float           _signal2;               // n-2 value of filter signal
    float           _ntchsig;               // current value of notch signal
    float           _ntchsig1;              // n-1 value of notch signal
    float           _ntchsig2;              // n-2 value of notch signal

    float           _last_requested_rate;       // Requested rate from last iteration, used to calculate rate change of requested rate
};
