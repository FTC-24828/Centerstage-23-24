package org.firstinspires.ftc.teamcode.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotionProfile {
    static private ElapsedTime timer = new ElapsedTime();

    public double max_acceleration, max_velocity;
    public double position, velocity, acceleration;

    public MotionProfile(double m_accel, double m_vel) {
        this.max_acceleration = m_accel;
        this.max_velocity = m_vel;
    }

    public void set(double m_accel, double m_vel) {
        this.max_acceleration = m_accel;
        this.max_velocity = m_vel;
    }

    public void update() {

    }
}
