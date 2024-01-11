package org.firstinspires.ftc.teamcode.common.hardware.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WServo;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.common.util.WMath;

import java.util.function.DoubleSupplier;

public class Intake implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();

    public enum ClawState {OPEN, CLOSED}
    private ClawState claw_right_state;
    private ClawState claw_left_state;

    public enum ClawSide {LEFT, RIGHT, BOTH}

    public enum WristState {SCORING, FOLD, FLAT}
    public DoubleSupplier wrist_angle;
    public double increment = 0;
    private double angle_offset;      //NOTE: TUNE IF CLAW ANGLE IS WRONG
    public WristState wrist_state = WristState.FOLD;
    public double arm_angle;

    public Intake() {
    }

    public void init(WServo wrist, WServo claw0, WServo claw1) {
        wrist.setDirection(Servo.Direction.REVERSE);
        angle_offset = robot.wrist.getWritingOffset();

        claw0.setDirection(Servo.Direction.REVERSE);
        claw1.setDirection(Servo.Direction.FORWARD);

        claw0.scaleRange(0.7, 0.9);
        claw1.scaleRange(0.7, 0.9);

        setClawState(ClawSide.BOTH, ClawState.CLOSED);

        wrist_angle = () -> (robot.wrist_actuator.getCurrentPosition() - angle_offset) * Math.PI / 2 ;
    }

    public void periodic() {
        //check pivot state and calculate pivot angle
        double target_position;
        switch (wrist_state) {
            case FLAT:
                target_position = angle_offset;
                break;

            case FOLD:
                target_position = 1.4;
                break;

            case SCORING:
                target_position = (WMath.twoPI / 3 - arm_angle) + increment;
                break;

            default:
                target_position = robot.wrist_actuator.getCurrentPosition() - angle_offset;
        }
        robot.wrist_actuator.setTargetPosition(target_position/2); //target_position should be from 1 to -1
    }

    public void read() {
        robot.wrist_actuator.read();
        this.arm_angle = robot.arm.arm_angle.getAsDouble();
    }

    public void write() {
        robot.wrist_actuator.write();
    }

    public void reset() {
        setWristState(WristState.FOLD);
        setClawState(ClawSide.BOTH, ClawState.CLOSED);
    }

    public void setClawState(@NonNull ClawSide side, @NonNull ClawState state) {
        double position = (state == ClawState.OPEN) ? 0.15 : 1;    //NOTE: CHANGE IF CLAW IS INVERTED
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
