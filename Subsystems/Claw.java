package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public Servo[] claw = new Servo[2];

    public void init (HardwareMap hmap) {
        Initialize initObj = new Initialize();

        initObj.initList(hmap, claw, Servo.class, "claw");

        claw[0].setDirection(Servo.Direction.FORWARD);
        claw[1].setDirection(Servo.Direction.FORWARD);
    }


}
