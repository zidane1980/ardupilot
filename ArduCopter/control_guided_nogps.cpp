#include "Copter.h"

/*
 * Init and run calls for guided_nogps flight mode
 */

// initialise guided_nogps controller
bool Copter::guided_nogps_init(bool ignore_checks)
{
#if FRAME_CONFIG == HELI_FRAME
    //keep compound-heli from using this mode
    if ((AP_Motors::motor_frame_class)g2.frame_class.get() == AP_Motors::MOTOR_FRAME_HELI_COMPOUND) {
        return false;
    }
#endif
    // start in angle control mode
    guided_angle_control_start();
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Copter::guided_nogps_run()
{
    // run angle controller
    guided_angle_control_run();
}

