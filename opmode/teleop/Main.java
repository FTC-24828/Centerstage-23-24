package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import org.firstinspires.ftc.teamcode.common.util.WMath;

@TeleOp (name = "MainTeleOp")
public class Main extends CommandOpMode {
    private final WRobot robot = WRobot.getInstance();

    private GamepadEx controller;

    private double loop_time = 0.0;

    private double INITIAL_YAW = Global.YAW_OFFSET;  //TODO LINK BETWEEN THE TWO PROGRAMS
    private boolean SLOW_MODE = false;
    private Vector2D local_vector;

    private ElapsedTime timer;

    @Override
    public void initialize() {
        super.reset();

        Global.IS_AUTO = false;
        Global.USING_DASHBOARD = false;
        Global.USING_IMU = true;
        Global.USING_WEBCAM = false;

        robot.addSubsystem(new Drivetrain(), new Intake(), new Arm());
        robot.init(hardwareMap, telemetry);

        controller = new GamepadEx(gamepad1);

        //toggle claw states
        controller.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ClawToggleCommand(Intake.ClawSide.BOTH));

        controller.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ClawToggleCommand(Intake.ClawSide.LEFT));

        controller.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ClawToggleCommand(Intake.ClawSide.RIGHT));

        //arm controls
        controller.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ConditionalCommand(
                        new ConditionalCommand(
                                new IntakeSequence(),
                                new IntermediateSequence(),
                                () -> Global.STATE == Global.State.INTERMEDIATE
                        ),
                        new InstantCommand(),
                        () -> Global.STATE != Global.State.INTAKING
                ));

        controller.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ConditionalCommand(
                        new ConditionalCommand(
                                new DepositSequence(),
                                new IntermediateSequence(),
                                () -> Global.STATE == Global.State.INTERMEDIATE
                        ),
                        new InstantCommand(),
                        () -> Global.STATE != Global.State.SCORING
                ));

        //slow mode
        controller.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> SLOW_MODE = true))
                .whenReleased(new InstantCommand(() -> SLOW_MODE = false));

        //yaw manual reset
        controller.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> INITIAL_YAW = robot.getYaw()));

        while (opModeInInit()) {
            telemetry.addLine("Initialization complete.");
            telemetry.update();
        }
    }


    @Override
    public void run() {
        if (timer == null) {
            timer = new ElapsedTime();
            robot.startIMUThread();
        }
        robot.read();

        local_vector = new Vector2D(controller.getLeftX(), controller.getLeftY(), WMath.wrapAngle(robot.getYaw() - INITIAL_YAW));
        if (SLOW_MODE) local_vector.scale(0.5);

        //left trigger gets precedent
        if (controller.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
            super.schedule(new ArmAdjustCommand(-5));
        }
        else if (controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            super.schedule(new ArmAdjustCommand(5));
        }

        super.run();
        robot.periodic();

        robot.drivetrain.move(local_vector, controller.getRightX() * (SLOW_MODE ? 0.5 : 1));

        double loop = System.nanoTime();
        telemetry.addData("Frequency", "%.2fhz", 1000000000 / (loop - loop_time));
        telemetry.addData("Voltage", "%.2f", robot.getVoltage());
        telemetry.addData("arm angle", "%.2f", Math.toDegrees(robot.arm.arm_angle.getAsDouble()));
        telemetry.addData("wrist angle", "%.2f", Math.toDegrees(robot.intake.wrist_angle.getAsDouble()));
        telemetry.addData("INITIAL YAW", "%.2f", INITIAL_YAW);
        telemetry.addData("State", Global.STATE);

        telemetry.update();

        loop_time = loop;
        robot.write();
        robot.clearBulkCache(Global.Hub.EXPANSION_HUB);
    }

    @Override
    public void reset() {
        super.reset();
        robot.reset();
        Global.resetGlobals();
    }
}