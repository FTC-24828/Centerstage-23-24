package org.firstinspires.ftc.teamcode.common.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotionProfile {
    static private ElapsedTime timer = new ElapsedTime();

    public double max_acceleration, max_deceleration, max_velocity;
    public double position, velocity, acceleration;

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

    public void update() {

    }
}
