# Guided Angle Control 400hz

mavlink sets:
  guided_angle_state.roll_in
  guided_angle_state.pitch_in
  guided_angle_state.yaw_cd
  guided_angle_state.yaw_rate_cd

ModeGuided::angle_control_run()
  attitude_control->input_euler_angle_roll_pitch_yaw(roll_in, pitch_in, yaw_in, true);

    AC_AttitudeControl::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw)
      _attitude_target_euler_angle = ...
      attitude_controller_run_quat()

    AC_AttitudeControl::attitude_controller_run_quat()
      attitude_controller_run_quat();
        _rate_target_ang_vel = AC_AttitudeControl::update_ang_vel_target_from_att_error(const Vector3f &attitude_error_rot_vec_rad)
          return _p_angle_role.kP * sqrt(attitude_error_rot_vec_rad)
      _rate_target_ang_vel = f(_attitude_target_euler_angle or _attitude_target_quat)

  pos_control->set_alt_target_from_climb_rate_ff(climb_rate_cms, G_Dt, false);
    AC_PosControl::set_alt_target_from_climb_rate_ff(float climb_rate_cms, float dt, bool force_descend)
      _vel_desired.z = constrain_float(climb_rate_cms, _vel_desired.z - vel_change_limit, _vel_desired.z + vel_change_limit);
      _pos_target.z += _vel_desired.z * dt;

  pos_control->update_z_controller();
    AC_PosControl::run_z_controller()
      thr_out = sqrt_controller(_pos_target.z)
      _attitude_control.set_throttle_out(thr_out, true, POSCONTROL_THROTTLE_CUTOFF_FREQ);

# Acro Mode 400 hz

ModeAcro::run()
  attitude_control->input_rate_bf_roll_pitch_yaw(target_roll, target_pitch, target_yaw);

    AC_AttitudeControl::input_rate_bf_roll_pitch_yaw(float roll_rate_bf_cds, float pitch_rate_bf_cds, float
     yaw_rate_bf_cds)
      // enforce acceleration maximums
      _attitude_target_quat = ...
      attitude_controller_run_quat();
        # See above

  attitude_control->set_throttle_out(get_pilot_desired_throttle(),
                                     false,
                                     copter.g.throttle_filt);
    AC_AttitudeControl_Multi::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
      _motors.set_throttle(throttle_in);
        in AP_Motors_Class.h
          set_throttle(float throttle_in) { _throttle_in = throttle_in; };   // range 0 ~ 1

      _motors.set_throttle_avg_max(get_throttle_avg_max(MAX(throttle_in, _throttle_in)));

# Copter::fast_loop 400 hz

ins.update();

void AC_AttitudeControl_Multi::rate_controller_run()
    // move throttle vs attitude mixing towards desired (called from here because this is conveniently called on every iteration)
    update_throttle_rpy_mix();

    Vector3f gyro_latest = _ahrs.get_gyro_latest();

    _motors.set_roll(get_rate_roll_pid().update_all(_rate_target_ang_vel.x, gyro_latest.x, _motors.limit.roll));
    _motors.set_roll_ff(get_rate_roll_pid().get_ff());

    _motors.set_pitch(get_rate_pitch_pid().update_all(_rate_target_ang_vel.y, gyro_latest.y, _motors.limit.pitch));
    _motors.set_pitch_ff(get_rate_pitch_pid().get_ff());

    _motors.set_yaw(get_rate_yaw_pid().update_all(_rate_target_ang_vel.z, gyro_latest.z, _motors.limit.yaw));
    _motors.set_yaw_ff(get_rate_yaw_pid().get_ff());

    in AP_Motors_Class.h
      set_roll(float roll_in) { _roll_in = roll_in; };        // range -1 ~ +1
      set_pitch(float pitch_in) { _pitch_in = pitch_in; };    // range -1 ~ +1
      set_yaw(float yaw_in) { _yaw_in = yaw_in; };            // range -1 ~ +1

  void AP_MotorsMulticopter::output()
      update_lift_max_from_batt_voltage();
      // calculate thrust
      output_armed_stabilizing();

        AP_MotorsMatrix::output_armed_stabilizing()
          const float compensation_gain = get_compensation_gain(); // compensation for battery voltage and altitude
          roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
          _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];
          _thrust_rpyt_out[i] = f(_thrust_rpyt_out[i], throttle_thrust)

      // apply any thrust compensation for the frame
      thrust_compensation();

      // convert rpy_thrust values to pwm
      output_to_motors();

      output_boost_throttle();
