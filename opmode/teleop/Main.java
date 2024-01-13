package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.DroneResetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.LaunchDroneCommand;
import org.firstinspires.ftc.teamcode.commands.telecommand.ArmAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.telecommand.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.commands.telecommand.DepositSequence;
import org.firstinspires.ftc.teamcode.commands.telecommand.IntakeSequence;
import org.firstinspires.ftc.teamcode.commands.telecommand.IntermediateSequence;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Drone;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;

@TeleOp (name = "MainTeleOp")
public class Main extends CommandOpMode {
    private final WRobot robot = WRobot.getInstance();

    private GamepadEx controller1;
    private GamepadEx controller2;

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

        robot.addSubsystem(new Drivetrain(), new Intake(), new Arm(), new Drone());
        robot.init(hardwareMap, telemetry);

        robot.arm.setArmState(Arm.ArmState.FLAT);
        robot.intake.setClawState(Intake.ClawSide.BOTH, Intake.ClawState.CLOSED);

        controller1 = new GamepadEx(gamepad1);
        controller2 = new GamepadEx(gamepad2);

        //toggle claw states
        controller1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ClawToggleCommand(robot, Intake.ClawSide.BOTH));

        controller1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ClawToggleCommand(robot, Intake.ClawSide.LEFT));

        controller1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ClawToggleCommand(robot, Intake.ClawSide.RIGHT));

        //arm controls
        controller1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ConditionalCommand(
                        new ConditionalCommand(
                                new IntakeSequence(robot),
                                new IntermediateSequence(robot),
                                () -> Global.STATE == Global.State.INTERMEDIATE
                        ),
                        new InstantCommand(),
                        () -> Global.STATE != Global.State.INTAKING
                ));

        controller1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ConditionalCommand(
                        new ConditionalCommand(
                                new DepositSequence(robot),
                                new IntermediateSequence(robot),
                                () -> Global.STATE == Global.State.INTERMEDIATE
                        ),
                        new InstantCommand(),
                        () -> Global.STATE != Global.State.SCORING
                ));

        //slow mode
        controller1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> SLOW_MODE = true))
                .whenReleased(new InstantCommand(() -> SLOW_MODE = false));

        controller1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new LaunchDroneCommand(robot));

        controller1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                        .whenPressed(new DroneResetCommand(robot));

        //yaw manual reset
        controller1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
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
            robot.startIMUThread(this);
        }
        robot.read();

        local_vector = new Vector2D(controller1.getLeftX(), controller1.getLeftY(), WMath.wrapAngle(robot.getYaw() - INITIAL_YAW));
        if (SLOW_MODE) local_vector.scale(0.5);

        //left trigger gets precedent
        if (controller1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
            super.schedule(new ArmAdjustCommand(robot, -3));
        }
        else if (controller1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
            super.schedule(new ArmAdjustCommand(robot, 3));
        }

        super.run();
        robot.periodic();

        robot.drivetrain.move(local_vector, controller1.getRightX() * (SLOW_MODE ? 0.5 : 1));
        robot.write();
        robot.clearBulkCache(Global.Hub.BOTH);

        double loop = System.nanoTime();
        telemetry.addData("Frequency", "%.2fhz", 1000000000 / (loop - loop_time));
        telemetry.addData("Voltage", "%.2f", robot.getVoltage());
        telemetry.addData("arm angle", "%.2f", Math.toDegrees(robot.arm.arm_angle.getAsDouble()));
        telemetry.addData("wrist angle", "%.2f", Math.toDegrees(robot.intake.wrist_angle.getAsDouble()));
        telemetry.addData("yaw", "%.2f", robot.getYaw() - INITIAL_YAW);
        telemetry.addData("State", Global.STATE);
        telemetry.update();
        loop_time = loop;
    }

    @Override
    public void reset() {
        super.reset();
        robot.reset();
        Global.resetGlobals();
    }
}