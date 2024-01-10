package org.firstinspires.ftc.teamcode.common.hardware.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.Pose;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;

import java.util.Arrays;

public class Drivetrain implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();
    public double[] wheel_speed = new double[4];
    private double[] prev_speed = new double[4];

    public void init (DcMotorEx[] motor) {
//      set drivetrain properties
        motor[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motor[1].setDirection(DcMotorSimple.Direction.FORWARD);
        motor[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motor[3].setDirection(DcMotorSimple.Direction.REVERSE);

        motor[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor[2].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor[3].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (int i=0; i<4; i++) {
            prev_speed[i] = 0;
        }
    }

    public void periodic() {

    }

    public void read() {
        if (Global.IS_AUTO) {
        }
    }

    public void write() {
//        if (Math.abs(wheel_speed[0] - prev_speed[0]) > 0.005) {
//            robot.motor[0].setPower(wheel_speed[0]);
//            prev_speed[0] = wheel_speed[0];
//        }
//        if (Math.abs(wheel_speed[1] - prev_speed[1]) > 0.005) {
//            robot.motor[1].setPower(wheel_speed[1]);
//            prev_speed[1] = wheel_speed[1];
//        }
//        if (Math.abs(wheel_speed[2]) < 0.005) {
//            robot.motor[2].setPower(wheel_speed[2]);
//            prev_speed[2] = wheel_speed[2];
//        }
//        if (Math.abs(wheel_speed[3] - prev_speed[3]) > 0.005) {
//            robot.motor[3].setPower(wheel_speed[3]);
//            prev_speed[3] = wheel_speed[3];
//        }

        robot.motor[0].setPower(wheel_speed[0]);
        robot.motor[1].setPower(wheel_speed[1]);
        robot.motor[2].setPower(wheel_speed[2]);
        robot.motor[3].setPower(wheel_speed[3]);
    }

    public void reset() {

    }

    public void move(Pose pose) {
        move(pose.x, pose.y, pose.z) ;
    }

    public void move(Vector2D v, double z) {
        move(v.x, v.y, z);
    }

    public void move(double x, double y, double z) {
        double angle = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double cos = Math.cos(angle - Math.PI/4);
        double sin = Math.sin(angle - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double[] speed = {
                power * sin / max,
                power * cos / max,
        };


        wheel_speed[0] = (WMath.clamp(speed[0] - z, -1.0, 1.0));
        wheel_speed[1] = (WMath.clamp(speed[1] - z, -1.0, 1.0));
        wheel_speed[2] = (WMath.clamp(speed[0] + z, -1.0, 1.0));
        wheel_speed[3] = (WMath.clamp(speed[1] + z, -1.0, 1.0));

        if (Global.IS_AUTO) {
            double correction = 12 / robot.getVoltage();
            for (int i=0; i<wheel_speed.length; i++) {
                wheel_speed[i] = Math.abs(wheel_speed[i]) < 0.01 ?
                        wheel_speed[i] * correction :
                        (wheel_speed[i] + Math.signum(wheel_speed[i]) * 0.05) * correction;
            }
        }
    }

    public String toString() {
        return "W0: " + wheel_speed[0] + "W1: " + wheel_speed[1] + "W2: " + wheel_speed[2] + "W3: " + wheel_speed[3];
    }
}
