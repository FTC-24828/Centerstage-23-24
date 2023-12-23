package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Other.Vector;

public class Drivetrain {
    private DcMotor[] motor = new DcMotor[4];

    public void init (HardwareMap hmap) {
        Initialize initObj = new Initialize();

        initObj.initList(hmap, motor, DcMotor.class, "motor");

//      set drivetrain properties
        motor[0].setDirection(DcMotorSimple.Direction.FORWARD);
        motor[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motor[2].setDirection(DcMotorSimple.Direction.FORWARD);
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

    public void move(Vector v, double z) {
        double angle = Math.atan2(v.y, -v.x);
        double power = Math.hypot(-v.x, v.y);

        double cos = Math.cos(angle - Math.PI/4);
        double sin = Math.sin(angle - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double[] speed = {
                power * sin / max,
                power * cos / max,
        };

        motor[0].setPower(Range.clip(speed[1] - z, -1.0, 1.0));
        motor[1].setPower(Range.clip(speed[0] + z, -1.0, 1.0));
        motor[2].setPower(Range.clip(speed[0] - z, -1.0, 1.0));
        motor[3].setPower(Range.clip(speed[1] + z, -1.0, 1.0));
    }

    public void move(double x, double y, double z) {
        double angle = Math.atan2(y, -x);
        double power = Math.hypot(-x, y);

        double cos = Math.cos(angle - Math.PI/4);
        double sin = Math.sin(angle - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double[] speed = {
                power * sin / max,
                power * cos / max,
        };

        motor[0].setPower(Range.clip(speed[1] - z, -1.0, 1.0));
        motor[1].setPower(Range.clip(speed[0] + z, -1.0, 1.0));
        motor[2].setPower(Range.clip(speed[0] - z, -1.0, 1.0));
        motor[3].setPower(Range.clip(speed[1] + z, -1.0, 1.0));
    }

    public void turn(double x) {
        motor[0].setPower(-x);
        motor[1].setPower(x);
        motor[2].setPower(-x);
        motor[3].setPower(x);
    }

    //TODO ADD A FAILSAFE FOR REORIENTING YAW AFTER AUTONOMOUS

    //returns x and y values relative to robot using the yaw angle and movement joystick
    public static Vector localOrientation(double x, double y, float z) {
        return new Vector (
                x * Math.sin(z + Math.PI/2) + y * Math.sin(z),
                x * Math.cos(z + Math.PI/2) + y * Math.cos(z)
        );
    }
}
