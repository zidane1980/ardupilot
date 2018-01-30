/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_MotorsHeli_RSC.h"

extern const AP_HAL::HAL& hal;

// init_servo - servo initialization on start-up
void AP_MotorsHeli_RSC::init_servo()
{
    // setup RSC on specified channel by default
    SRV_Channels::set_aux_channel_default(_aux_fn, _default_channel);
}

// set_power_output_range
void AP_MotorsHeli_RSC::set_throttle_curve(float thrcrv_0, float thrcrv_25, float thrcrv_50, float thrcrv_75, float thrcrv_100, uint16_t slewrate)
{
    _rsc_thrcrv_0 = thrcrv_0;
    _rsc_thrcrv_25 = thrcrv_25;
    _rsc_thrcrv_50 = thrcrv_50;
    _rsc_thrcrv_75 = thrcrv_75;
    _rsc_thrcrv_100 = thrcrv_100;
    _power_slewrate = slewrate;
}

// output - update value to send to ESC/Servo
void AP_MotorsHeli_RSC::output(RotorControlState state)
{
    float dt;
    uint64_t now = AP_HAL::micros64();
    float last_control_output = _control_output;
    
    if (_last_update_us == 0) {
        _last_update_us = now;
        dt = 0.001f;
    } else {
        dt = 1.0e-6f * (now - _last_update_us);
        _last_update_us = now;
    }
    
    switch (state){
        case ROTOR_CONTROL_STOP:
            // set rotor ramp to decrease speed to zero, this happens instantly inside update_rotor_ramp()
            update_rotor_ramp(0.0f, dt);

            // control output forced to zero
            _control_output = 0.0f;
            break;

        case ROTOR_CONTROL_IDLE:
            // set rotor ramp to decrease speed to zero
            update_rotor_ramp(0.0f, dt);

            // set rotor control speed to idle speed parameter, this happens instantly and ignore ramping
            _control_output = _idle_output;
            break;

        case ROTOR_CONTROL_ACTIVE:
            // set main rotor ramp to increase to full speed
            update_rotor_ramp(1.0f, dt);

            if ((_control_mode == ROTOR_CONTROL_MODE_SPEED_PASSTHROUGH) || (_control_mode == ROTOR_CONTROL_MODE_SPEED_SETPOINT)) {
                // set control rotor speed to ramp slewed value between idle and desired speed
                _control_output = _idle_output + (_rotor_ramp_output * (_desired_speed - _idle_output));
            } else if (_control_mode == ROTOR_CONTROL_MODE_OPEN_LOOP_POWER_OUTPUT) {
                // throttle output from throttle curve based on collective position
                    
                    float desired_throttle = calculate_desired_throttle(_collective_in, _rsc_thrcrv_0, _rsc_thrcrv_25, _rsc_thrcrv_50, _rsc_thrcrv_75, _rsc_thrcrv_100);
                    _control_output = _idle_output + (_rotor_ramp_output * (desired_throttle - _idle_output));
            }
            break;
    }

    // update rotor speed run-up estimate
    update_rotor_runup(dt);

    if (_power_slewrate > 0) {
        // implement slew rate for throttle
        float max_delta = dt * _power_slewrate * 0.01f;
        _control_output = constrain_float(_control_output, last_control_output-max_delta, last_control_output+max_delta);
    }
    
    // output to rsc servo
    write_rsc(_control_output);
}

// update_rotor_ramp - slews rotor output scalar between 0 and 1, outputs float scalar to _rotor_ramp_output
void AP_MotorsHeli_RSC::update_rotor_ramp(float rotor_ramp_input, float dt)
{
    // sanity check ramp time
    if (_ramp_time <= 0) {
        _ramp_time = 1;
    }

    // ramp output upwards towards target
    if (_rotor_ramp_output < rotor_ramp_input) {
        // allow control output to jump to estimated speed
        if (_rotor_ramp_output < _rotor_runup_output) {
            _rotor_ramp_output = _rotor_runup_output;
        }
        // ramp up slowly to target
        _rotor_ramp_output += (dt / _ramp_time);
        if (_rotor_ramp_output > rotor_ramp_input) {
            _rotor_ramp_output = rotor_ramp_input;
        }
    }else{
        // ramping down happens instantly
        _rotor_ramp_output = rotor_ramp_input;
    }
}

// update_rotor_runup - function to slew rotor runup scalar, outputs float scalar to _rotor_runup_ouptut
void AP_MotorsHeli_RSC::update_rotor_runup(float dt)
{
    // sanity check runup time
    if (_runup_time < _ramp_time) {
        _runup_time = _ramp_time;
    }
    if (_runup_time <= 0 ) {
        _runup_time = 1;
    }

    // ramp speed estimate towards control out
    float runup_increment = dt / _runup_time;
    if (_rotor_runup_output < _rotor_ramp_output) {
        _rotor_runup_output += runup_increment;
        if (_rotor_runup_output > _rotor_ramp_output) {
            _rotor_runup_output = _rotor_ramp_output;
        }
    }else{
        _rotor_runup_output -= runup_increment;
        if (_rotor_runup_output < _rotor_ramp_output) {
            _rotor_runup_output = _rotor_ramp_output;
        }
    }

    // update run-up complete flag

    // if control mode is disabled, then run-up complete always returns true
    if ( _control_mode == ROTOR_CONTROL_MODE_DISABLED ){
        _runup_complete = true;
        return;
    }

    // if rotor ramp and runup are both at full speed, then run-up has been completed
    if (!_runup_complete && (_rotor_ramp_output >= 1.0f) && (_rotor_runup_output >= 1.0f)) {
        _runup_complete = true;
    }
    // if rotor speed is less than critical speed, then run-up is not complete
    // this will prevent the case where the target rotor speed is less than critical speed
    if (_runup_complete && (get_rotor_speed() <= _critical_speed)) {
        _runup_complete = false;
    }
}

// get_rotor_speed - gets rotor speed either as an estimate, or (ToDO) a measured value
float AP_MotorsHeli_RSC::get_rotor_speed() const
{
    // if no actual measured rotor speed is available, estimate speed based on rotor runup scalar.
    return _rotor_runup_output;
}

// write_rsc - outputs pwm onto output rsc channel
// servo_out parameter is of the range 0 ~ 1
void AP_MotorsHeli_RSC::write_rsc(float servo_out)
{
    if (_control_mode == ROTOR_CONTROL_MODE_DISABLED){
        // do not do servo output to avoid conflicting with other output on the channel
        // ToDo: We should probably use RC_Channel_Aux to avoid this problem
        return;
    } else {
        SRV_Channels::set_output_scaled(_aux_fn, (uint16_t) (servo_out * 1000));
    }
}

    // calculate_desired_throttle - uses throttle curve and collective input to determine throttle setting
float AP_MotorsHeli_RSC::calculate_desired_throttle(float collective_in, float thrcrv_0, float thrcrv_25, float thrcrv_50, float thrcrv_75, float thrcrv_100)
{
    std::vector<std::vector<double>> thrcrv_poly(5, std::vector<double>(4));
    std::vector<double> thrcrv(5);
    double throttle = 0.0f;
    double inpt, a, b;
    thrcrv[0] = thrcrv_0;
    thrcrv[1] = thrcrv_25;
    thrcrv[2] = thrcrv_50;
    thrcrv[3] = thrcrv_75;
    thrcrv[4] = thrcrv_100;

    thrcrv_poly = splinterp(thrcrv);

    inpt = (double)collective_in * 4.0f + 1.0f;
    if (collective_in < 0.25){
        a = inpt - 1.0f;
        b = 1.0f - inpt + 1.0f;
        throttle = thrcrv_poly[0][0] * a + thrcrv_poly[0][1] * b + thrcrv_poly[0][2] * (pow(a,3.0f) - a) / 6.0f + thrcrv_poly[0][3] * (pow(b,3.0f) - b) / 6.0f;
    } else if (collective_in < 0.5){
        a = inpt - 2.0f;
        b = 2.0f - inpt + 1.0f;
        throttle = thrcrv_poly[1][0] * a + thrcrv_poly[1][1] * b + thrcrv_poly[1][2] * (pow(a,3.0f) - a) / 6.0f + thrcrv_poly[1][3] * (pow(b,3.0f) - b) / 6.0f;
    } else if (collective_in < 0.75){
        a = inpt - 3.0f;
        b = 3.0f - inpt + 1.0f;
        throttle = thrcrv_poly[2][0] * a + thrcrv_poly[2][1] * b + thrcrv_poly[2][2] * (pow(a,3.0f) - a) / 6.0f + thrcrv_poly[2][3] * (pow(b,3.0f) - b) / 6.0f;
    } else{
        a = inpt - 4.0f;
        b = 4.0f - inpt + 1.0f;
        throttle = thrcrv_poly[3][0] * a + thrcrv_poly[3][1] * b + thrcrv_poly[3][2] * (pow(a,3.0f) - a) / 6.0f + thrcrv_poly[3][3] * (pow(b,3.0f) - b) / 6.0f;
    }
    
    throttle = constrain_float(throttle, 0.0f, 1.0f);
    return throttle;

}

