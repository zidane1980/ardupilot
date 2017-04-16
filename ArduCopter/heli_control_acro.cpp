#include "Copter.h"

#if FRAME_CONFIG == HELI_FRAME
/*
 * Init and run calls for acro flight mode for trad heli
 */

// heli_acro_init - initialise acro controller
bool Copter::heli_acro_init(bool ignore_checks)
{
    // if heli is equipped with a flybar, then tell the attitude controller to pass through controls directly to servos
    attitude_control->use_flybar_passthrough(motors->has_flybar(), motors->supports_yaw_passthrough());

    motors->set_acro_tail(true);
    
    // set stab collective false to use full collective pitch range
    input_manager.set_use_stab_col(false);

    // always successfully enter acro
    return true;
}

// heli_acro_run - runs the acro controller
// should be called at 100hz or more
void Copter::heli_acro_run()
{
    float target_roll, target_pitch, target_yaw;
    float pilot_throttle_scaled;
    AcroModeState acro_state;
    
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

    // Acro State Machine Determination
    if (!motors->get_interlock() || !motors->rotor_runup_complete() || !motors->rotor_speed_above_critical()) {
        acro_state = Acro_MotorStopped;
    } else if (ap.land_complete && !motor_at_lower_limit && (!accel_stationary || !descent_rate_low)) {
        acro_state = Acro_Takeoff;
    } else if (ap.land_complete) {
        acro_state = Acro_Landed;
    } else {
        acro_state = Acro_Flying;
    }

    // writes an event when state of aircraft changes
    if (acro_state != acro_state_m1) {
        if (acro_state == Acro_MotorStopped) {Log_Write_Event(DATA_STATE_MOTORSTOPPED);}
        if (acro_state == Acro_Landed) {Log_Write_Event(DATA_STATE_LANDED);}
        if (acro_state == Acro_Takeoff) {Log_Write_Event(DATA_STATE_TAKEOFF);}
        if (acro_state == Acro_Flying) {Log_Write_Event(DATA_STATE_FLYING);}
        acro_state_m1 = acro_state;
    }

    // Acro State Machine
    switch (acro_state) {

    case Acro_MotorStopped:
        attitude_control->set_attitude_target_to_current_attitude();
        attitude_control->reset_rate_controller_I_terms();
        break;
    case Acro_Landed:
        attitude_control->leak_roll_target_to_current_attitude();
        attitude_control->leak_pitch_target_to_current_attitude();
        attitude_control->leak_yaw_target_to_current_heading();
        break;
    case Acro_Takeoff:
        attitude_control->leak_roll_target_to_current_attitude();
        attitude_control->leak_pitch_target_to_current_attitude();
        attitude_control->leak_yaw_target_to_current_heading();
        set_land_complete(false);
        break;
    case Acro_Flying:
        // Leak attitude if we are not moving fast
        if (!heli_flags.dynamic_flight) {
            attitude_control->leak_roll_target_to_current_attitude();
            attitude_control->leak_pitch_target_to_current_attitude();
            attitude_control->leak_yaw_target_to_current_heading();
        }
        break;
    }

    if (!motors->has_flybar()){
        // convert the input to the desired body frame rate
        get_pilot_desired_angle_rates(channel_roll->get_control_in(), channel_pitch->get_control_in(), channel_yaw->get_control_in(), target_roll, target_pitch, target_yaw);

        if (motors->supports_yaw_passthrough()) {
            // if the tail on a flybar heli has an external gyro then
            // also use no deadzone for the yaw control and
            // pass-through the input direct to output.
            target_yaw = channel_yaw->get_control_in_zero_dz();
        }

        // run attitude controller
        attitude_control->input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);
    }else{
        /*
          for fly-bar passthrough use control_in values with no
          deadzone. This gives true pass-through.
         */
        float roll_in = channel_roll->get_control_in_zero_dz();
        float pitch_in = channel_pitch->get_control_in_zero_dz();
        float yaw_in;
        
        if (motors->supports_yaw_passthrough()) {
            // if the tail on a flybar heli has an external gyro then
            // also use no deadzone for the yaw control and
            // pass-through the input direct to output.
            yaw_in = channel_yaw->get_control_in_zero_dz();
        } else {
            // if there is no external gyro then run the usual
            // ACRO_YAW_P gain on the input control, including
            // deadzone
            yaw_in = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        }

        // run attitude controller
        attitude_control->passthrough_bf_roll_pitch_rate_yaw(roll_in, pitch_in, yaw_in);
    }

    // get pilot's desired throttle
    pilot_throttle_scaled = input_manager.get_pilot_desired_collective(channel_throttle->get_control_in());

    // output pilot's throttle without angle boost
    attitude_control->set_throttle_out(pilot_throttle_scaled, false, g.throttle_filt);
}

#endif  //HELI_FRAME
