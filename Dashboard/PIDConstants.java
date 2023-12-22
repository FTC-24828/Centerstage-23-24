package org.firstinspires.ftc.teamcode.Dashboard;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
public class PIDConstants {
    public static double Kp = 0.001;  // Proportional gain
    public static double Ki = 0.02;   // Integral gain
    public static double Kd = 0.0001;   // Derivative gain
    public static double Kf = 0.2;
    public static double lim = 1;
    public static double maxErr = 10;
}