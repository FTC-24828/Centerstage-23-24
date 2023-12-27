package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.wrappers.WServo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.WSubsystem;

import java.util.function.DoubleSupplier;

public class Intake extends WSubsystem {
    private final Robot robot = Robot.getInstance();

    public enum ClawState {OPEN, CLOSED}
    private ClawState claw_right_state;
    private ClawState claw_left_state;

    public enum ClawSide {LEFT, RIGHT, BOTH}

    public enum WristState {SCORING, FOLD, FLAT}
    public DoubleSupplier wrist_angle;
    private double angle_offset;      //NOTE: TUNE IF CLAW ANGLE IS WRONG
    private WristState wrist_state = WristState.FOLD;

    public Intake() {
        wrist_angle = () -> (robot.wrist_actuator.getCurrentPosition() - angle_offset) * Math.PI / 2 ;
    }

    public void init(WServo wrist, WServo claw0, WServo claw1) {
        wrist.setDirection(Servo.Direction.FORWARD);
        angle_offset = robot.wrist.getOffset();

        claw0.setDirection(Servo.Direction.REVERSE);
        claw1.setDirection(Servo.Direction.FORWARD);

        claw0.scaleRange(0.7, 0.9);
        claw1.scaleRange(0.7, 0.9);

        setClawState(ClawSide.BOTH, ClawState.OPEN);
    }

    public void periodic() {
        //check pivot state and calc pivot angle
        double target_position;
        switch (wrist_state) {
            case FLAT:
                target_position = 0.2;
                break;

            case FOLD:
                target_position = 1;
                break;

            case SCORING:
                target_position = (2 * Math.PI / 3 - robot.arm.arm_angle.getAsDouble()) - angle_offset;
                break;

            default:
                target_position = robot.wrist_actuator.getCurrentPosition() - angle_offset;
        }
        robot.wrist_actuator.setTargetPosition(target_position/2); //target_position should be from 1 to -1
    }

    public void read() {
        robot.wrist_actuator.read();
    }

    public void write() {
        robot.wrist_actuator.write();
    }

    public void reset() {

    }

    public void setClawState(@NonNull ClawSide side, @NonNull ClawState state) {
        double position = (state == ClawState.OPEN) ? 1 : 0;    //NOTE: CHANGE IF CLAW IS INVERTED
        switch (side) {
            case BOTH:
                claw_left_state = state;
                robot.claw_left.setPosition(position);
                claw_right_state = state;
                robot.claw_right.setPosition(position);
                break;

            case LEFT:
                claw_left_state = state;
                robot.claw_left.setPosition(position);
                break;

            case RIGHT:
                claw_right_state = state;
                robot.claw_right.setPosition(position);
                break;
        }
    }

    public ClawState getClawState(@NonNull ClawSide side) {
        switch (side) {
            case LEFT:
                return claw_left_state;

            case RIGHT:
                return claw_right_state;

            //return OPEN if claws are in different states
            case BOTH:
                if (claw_left_state != claw_right_state) return ClawState.OPEN;
                return claw_left_state;

            default:
                throw new RuntimeException("getClawState method called with NULL");
        }
    }

    public void setWristState(@NonNull WristState state) {
        this.wrist_state = state;
    }
}
