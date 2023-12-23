package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
public class Controller {

    private Gamepad gamepad;

    private class Buttons {
        boolean a, b, x, y,
                back, start, guide,
                dpad_down, dpad_left, dpad_right, dpad_up,
                left_bumper, left_stick_button, right_stick_button, right_bumper;
        float	left_stick_x;
        float	left_stick_y;
        float	left_trigger;
        float	right_stick_x;
        float	right_stick_y;
        float	right_trigger;
    }

    public Controller(Gamepad gp) {
        this.gamepad = gp;
    }

//    public void update() {
//        gamepad.
//    }
}
