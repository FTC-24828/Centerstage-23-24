package org.firstinspires.ftc.teamcode.common.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WServo;
import org.firstinspires.ftc.teamcode.common.hardware.wrappers.WSubsystem;

public class Hang implements WSubsystem {
    private final WRobot robot = WRobot.getInstance();

    public enum HangState {HANGING, STOPPED, RETRACT}

    public double power = 0;
    public HangState hang_state = HangState.STOPPED;

    public void init(DcMotorEx winch0, DcMotorEx winch1, WServo hook) {
        winch0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winch0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winch0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch0.setDirection(DcMotorSimple.Direction.FORWARD);

        winch1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        winch1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winch1.setDirection(DcMotorSimple.Direction.REVERSE);

        hook.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void periodic() {
        robot.hang_actuator.setPower(power);
    }

    @Override
    public void read() {
        robot.arm_actuator.read();
    }

    @Override
    public void write() {
        robot.hang_actuator.write();
    }

    @Override
    public void reset() {
        robot.hang_actuator.setPower(0);
    }

    public void setHangPower(double power) {
        this.power = power;
    }

    public void setHangState(HangState state) {
        this.hang_state = state;
    }

    public void setHookPosition(double position) {
        robot.hook.setPosition(position);
    }

    public void setPowerMotor(double power) {
        robot.hang_motor0.setPower(power);
        robot.hang_motor1.setPower(power);
    }
}
