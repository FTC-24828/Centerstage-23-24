package org.firstinspires.ftc.teamcode.common.hardware.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;

public class Drivetrain implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();
    public double[] wheel_speed = new double[4];

    public void init (DcMotorEx[] motor) {
//      set drivetrain properties
        motor[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motor[1].setDirection(DcMotorSimple.Direction.FORWARD);
        motor[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motor[3].setDirection(DcMotorSimple.Direction.REVERSE);

        motor[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void periodic() {

    }

    public void read() {
        if (Global.IS_AUTO) {

        }
    }

    public void write() {
        for (int i = 0; i < 4; i++) {
            robot.motor[i].setPower(wheel_speed[i]);
        }
    }

    public void reset() {

    }

    public void move(Vector2D v, double z) {
        double angle = Math.atan2(v.y, v.x);
        double power = Math.hypot(v.x, v.y);

        double cos = Math.cos(angle - Math.PI/4);
        double sin = Math.sin(angle - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double[] speed = {
                power * sin / max,
                power * cos / max,
        };

        wheel_speed[0] = (WMath.clamp(speed[1] - z, -1.0, 1.0));
        wheel_speed[1] = (WMath.clamp(speed[0] - z, -1.0, 1.0));
        wheel_speed[2] = (WMath.clamp(speed[1] + z, -1.0, 1.0));
        wheel_speed[3] = (WMath.clamp(speed[0] + z, -1.0, 1.0));
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
    }

    //TODO ADD A FAILSAFE FOR REORIENTING YAW AFTER AUTONOMOUS


}
