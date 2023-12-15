package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Arm {
    public DcMotor arm;
    public Servo wrist;
    public Servo[] claw = new Servo[2];

    public void init (HardwareMap hmap) {
        Initialize initObj = new Initialize();

        initObj.initItem(hmap, arm, DcMotor.class, "arm");
        initObj.initItem(hmap, wrist, Servo.class, "wrist");
        initObj.initList(hmap, claw, Servo.class, "claw");

        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wrist.setDirection(Servo.Direction.FORWARD);
//        wrist.setPosition();

        claw[0].setDirection(Servo.Direction.FORWARD);
        claw[1].setDirection(Servo.Direction.FORWARD);
//        claw[0].setPosition();
    }


}
