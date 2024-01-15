package org.firstinspires.ftc.teamcode.common.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotionProfile {
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime dt_timer = new ElapsedTime();
    private double start_time = 0;

    public double max_acceleration, max_deceleration, max_velocity;
    public double prev_position, velocity, acceleration;

    public MotionProfile(double m_accel, double m_decel, double m_vel) {
        this.max_acceleration = m_accel;
        this.max_deceleration = m_decel;
        this.max_velocity = m_vel;
    }

    public MotionProfile(double m_accel, double m_vel) {
        this(m_accel, m_accel, m_vel);
    }

    public void set(double m_accel, double m_decel, double m_vel) {
        this.max_acceleration = m_accel;
        this.max_deceleration = m_decel;
        this.max_velocity = m_vel;
    }

    public double update(double current, double target, double tolerance) {
        double error = target - current;
        if (error < tolerance) {
            start_time = 0;
            return 0;
        }

        if (start_time == 0) start_time = timer.seconds();
        double elapsed_time = timer.seconds() - start_time;
        double dt = dt_timer.seconds();
        double theoretical_velocity_max = Math.sqrt(2.0 * error / (1/max_acceleration + 1/max_deceleration));
        double total_time = 2.0 * error / theoretical_velocity_max;

        double max_acceleration_t = max_velocity / max_acceleration;
        double max_deceleration_t = max_velocity / max_deceleration;

        velocity = (current - prev_position) / dt;
        acceleration = 2.0 * (current - prev_position) / (dt * dt);
        double deceleration_time_squared = (max_deceleration_t - total_time + elapsed_time) * (max_deceleration_t - total_time + elapsed_time);

        //if the error is not large enough to reach max velocity
        if (theoretical_velocity_max < max_velocity) {
            if (elapsed_time < theoretical_velocity_max / max_acceleration)
                return 0.5 * max_acceleration * elapsed_time * elapsed_time;
            else {
                return error - 0.5 * max_deceleration_t * deceleration_time_squared;
            }
        }

        //check if we are in accelerating period
        if (velocity < max_velocity && elapsed_time < max_acceleration_t) {
            return 0.5 * max_acceleration * elapsed_time * elapsed_time;
        }

        //check if we are in decelerating period
        else if (velocity < max_velocity && elapsed_time < total_time) {
            return error - 0.5 * max_deceleration_t * deceleration_time_squared;
        }

        //maintain current velocity
        

        dt_timer.reset();
        return error;
    }
}
