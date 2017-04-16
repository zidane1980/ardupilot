#include "Copter.h"

#if FRAME_CONFIG == HELI_FRAME
/*
 * Init and run calls for stabilize flight mode for trad heli
 */

// stabilize_init - initialise stabilize controller
bool Copter::heli_stabilize_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control->set_alt_target(0);

    // set stab collective true to use stabilize scaled collective pitch range
    input_manager.set_use_stab_col(true);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::heli_stabilize_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;
    StabilizeModeState stabilize_state;

    // Tradheli should not reset roll, pitch, yaw targets when motors are not runup, because
    // we may be in autorotation flight.  These should be reset only when transitioning from disarmed
    // to armed, because the pilot will have placed the helicopter down on the landing pad.  This is so
    // that the servos move in a realistic fashion while disarmed for operational checks.
    // Also, unlike multicopters we do not set throttle (i.e. collective pitch) to zero so the swash servos move
    
    // check that collective pitch is on lower limit (should be constrained by LAND_COL_MIN)
    bool motor_at_lower_limit = motors->limit.throttle_lower;

    // check that the airframe is not accelerating (not falling or breaking after fast forward flight)
    bool accel_stationary = (land_accel_ef_filter.get().length() <= LAND_DETECTOR_ACCEL_MAX);

    // check that vertical speed is within 1m/s of zero
    bool descent_rate_low = fabsf(inertial_nav.get_velocity_z()) < 100;

    // Stabilize State Machine Determination
    if (!motors->get_interlock() || !motors->rotor_runup_complete() || !motors->rotor_speed_above_critical()) {
        stabilize_state = Stabilize_MotorStopped;
    } else if (ap.land_complete && !motor_at_lower_limit && (!accel_stationary || !descent_rate_low)) {
        stabilize_state = Stabilize_Takeoff;
    } else if (ap.land_complete) {
        stabilize_state = Stabilize_Landed;
    } else {
        stabilize_state = Stabilize_Flying;
    }

    // writes an event when state of aircraft changes
    if (stabilize_state != stabilize_state_m1) {
        if (stabilize_state == Stabilize_MotorStopped) {Log_Write_Event(DATA_STATE_MOTORSTOPPED);}
        if (stabilize_state == Stabilize_Landed) {Log_Write_Event(DATA_STATE_LANDED);}
        if (stabilize_state == Stabilize_Takeoff) {Log_Write_Event(DATA_STATE_TAKEOFF);}
        if (stabilize_state == Stabilize_Flying) {Log_Write_Event(DATA_STATE_FLYING);}
        stabilize_state_m1 = stabilize_state;
    }

    // Stabilize State Machine
    switch (stabilize_state) {

    case Stabilize_MotorStopped:
        attitude_control->set_yaw_target_to_current_heading();
        attitude_control->reset_rate_controller_I_terms();
        break;
    case Stabilize_Landed:
        attitude_control->leak_yaw_target_to_current_heading();
        break;
    case Stabilize_Takeoff:
        attitude_control->leak_yaw_target_to_current_heading();
        set_land_complete(false);
        break;
    case Stabilize_Flying:

        break;
    }

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    pilot_throttle_scaled = input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // output pilot's throttle - note that TradHeli does not used angle-boost
    attitude_control->set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
}

#endif  //HELI_FRAME
