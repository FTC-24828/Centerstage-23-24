package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public enum State {OPENED, CLOSED}
    private Servo[] claw = new Servo[2];
    public State[] currentState = {State.OPENED, State.OPENED};

    public void init (HardwareMap hmap) {
        Initialize initObj = new Initialize();

        initObj.initList(hmap, claw, Servo.class, "claw");

        claw[0].setDirection(Servo.Direction.REVERSE);
        claw[1].setDirection(Servo.Direction.FORWARD);

        claw[0].scaleRange(0.7, 0.9);
        claw[1].scaleRange(0.7, 0.9);

        claw[0].setPosition(0);
        claw[1].setPosition(0);
    }

    public void setState(int id, State state) {
        if (state == State.CLOSED) {
            currentState[id] = state;
            claw[id].setPosition(1);
        }
        else {
            currentState[id] = state;
            claw[id].setPosition(0);
        }
    }

    public void switchState(int id) {
        switch (currentState[id]) {
            case OPENED:
                setState(id, State.CLOSED); break;
            case CLOSED:
                setState(id, State.OPENED); break;
        }
    }

    public double getPosition(int id) {
        return claw[id].getPosition();
    }
}
