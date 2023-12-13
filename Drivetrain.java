package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Drivetrain {
    public DcMotor[] motor = new DcMotor[4];

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

    public void move(double x, double y) {
        motor[0].setPower(Range.clip(y-x, -1.0, 1.0));
        motor[1].setPower(Range.clip(y+x, -1.0, 1.0));
        motor[2].setPower(Range.clip(y+x, -1.0, 1.0));
        motor[3].setPower(Range.clip(y-x, -1.0, 1.0));
    }

    public void turn(double x) {
        motor[0].setPower(-x);
        motor[1].setPower(x);
        motor[2].setPower(-x);
        motor[3].setPower(x);
    }
}
