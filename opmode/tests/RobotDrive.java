package org.firstinspires.ftc.teamcode.opmode.tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.commands.telecommand.ArmAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.telecommand.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.commands.telecommand.DepositSequence;
import org.firstinspires.ftc.teamcode.commands.telecommand.IntakeSequence;
import org.firstinspires.ftc.teamcode.commands.telecommand.IntermediateSequence;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;

@TeleOp(name = "MainTeleOp - robot centric")
public class RobotDrive extends CommandOpMode{

    private final WRobot robot = WRobot.getInstance();

    private GamepadEx controller;

    private double loop_time = 0.0;

    public double INITIAL_YAW;          //TODO LINK BETWEEN THE TWO PROGRAMS
    public Vector2D local_vector;

    @Override
    public void initialize() {
        super.reset();

        Global.IS_AUTO = false;
        Global.USING_DASHBOARD = false;
        Global.USING_IMU = true;

        robot.addSubsystem(new Drivetrain(), new Intake(), new Arm());
        robot.init(hardwareMap, telemetry);
        telemetry.addLine("robot reset");
        robot.resetYaw();

        controller = new GamepadEx(gamepad1);

        //toggle claw states
        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ClawToggleCommand(Intake.ClawSide.BOTH));

        controller.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ClawToggleCommand(Intake.ClawSide.LEFT));

        controller.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ClawToggleCommand(Intake.ClawSide.RIGHT));

        //arm controls
        controller.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ConditionalCommand(
                        new IntermediateSequence(),
                        new IntakeSequence(),
                        () -> Global.IS_INTAKING && !Global.IS_SCORING
                ));

        controller.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ConditionalCommand(
                        new IntermediateSequence(),
                        new ParallelCommandGroup(
                                new WristCommand(Intake.WristState.FLAT),
                                new DepositSequence()
                        ),
                        () -> Global.IS_SCORING && !Global.IS_INTAKING
                ));

        //slow mode
        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> local_vector.scale(0.5)));

        while (opModeInInit()) {
            telemetry.addLine("Initialization complete.");
            telemetry.update();
        }
    }


    @Override
    public void run() {
        robot.read();
        super.run();

        local_vector = Vector2D.toLocalOrientation(controller.getLeftX(), controller.getLeftY(), robot.yaw);
        robot.drivetrain.move(local_vector, controller.getRightX());

        //left trigger gets precedent
        if (controller.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            super.schedule(new ArmAdjustCommand(-10));
        }
        else if (controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            super.schedule(new ArmAdjustCommand(10));
        }

        robot.periodic();

        double loop = System.nanoTime();
        telemetry.addData("Frequency", 1000000000 / (loop - loop_time) + "hz");
        telemetry.addData("Voltage", robot.getVoltage());
        telemetry.addData("arm angle", Math.toDegrees(robot.arm.arm_angle.getAsDouble()));
        telemetry.addData("wrist angle", Math.toDegrees(robot.intake.wrist_angle.getAsDouble()));
        telemetry.addData("Yaw", robot.yaw);
        telemetry.addData("increment", robot.arm.increment);
        telemetry.update();

        loop_time = loop;
        robot.write();
        robot.clearBulkCache(Global.Hub.EXPANSION_HUB);
    }

    @Override
    public void reset() {
        super.reset();
        robot.reset();
    }
}
