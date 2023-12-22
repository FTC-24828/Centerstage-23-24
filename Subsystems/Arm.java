package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Arm {
    public DcMotor[] lift = new DcMotor[1];
    public Servo[] wrist = new Servo[1];

    public void init (HardwareMap hmap) {
        Initialize initObj = new Initialize();

        initObj.initItem(hmap, lift, DcMotor.class, "motor4");
//        initObj.initItem(hmap, wrist, Servo.class, "wrist");

        lift[0].setDirection(DcMotorSimple.Direction.FORWARD);
        lift[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        wrist.setDirection(Servo.Direction.FORWARD);
////        wrist.setPosition();

    }

    void setPower(double i) { lift[0].setPower(i); }

    int getPosition() { return lift[0].getCurrentPosition(); }


}
