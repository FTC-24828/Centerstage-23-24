package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.controllers.Feedforward;
import org.firstinspires.ftc.teamcode.controllers.PIDF;
import org.firstinspires.ftc.teamcode.hardware.Global;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.wrappers.WSubsystem;
import org.firstinspires.ftc.teamcode.util.WMath;

import java.util.function.DoubleSupplier;


public class Arm extends WSubsystem {
    private final Robot robot = Robot.getInstance();

    public enum ArmState { SCORING, FLAT }
    private ArmState arm_state = ArmState.FLAT;

    public DoubleSupplier arm_angle;
    public double target_position;
    private double increment;

    //controllers
    private final PIDF armController = new PIDF(0.001, 0.02, 0.0001, 0.2, 1, 3);
    private final Feedforward armSupport = new Feedforward(0.07);

    public Arm() {
        arm_angle = () -> robot.arm_actuator.getCurrentPosition() / Global.MOTOR_TPR * Math.PI;
        target_position = robot.arm_actuator.getCurrentPosition();
    }

    public void init (DcMotorEx lift) {
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void periodic() {
        switch (arm_state) {
            case FLAT:
                target_position = 0;
                break;

            case SCORING:
                target_position = (double) 4 * Global.MOTOR_TPR / 5 + increment;
        }

        double power = armController.calculate(robot.arm_actuator.getCurrentPosition(), target_position) +
                armSupport.calculate(Math.cos(arm_angle.getAsDouble())) * ((arm_state == ArmState.FLAT) ? 0 : 1);
        robot.arm_actuator.setPower(power);
    }

    public void read() {
        robot.arm_actuator.read();
    }

    public void write() {
        robot.arm_actuator.write();
    }

    public void reset() {

    }

    public ArmState getArmState() {
        return arm_state;
    }

    public void setArmState(ArmState state) {
        arm_state = state;
    }

    public void incrementHeight(double increment) {
        this.increment -= increment;
        this.increment = WMath.clamp(this.increment, -20, 10);
    }
}
