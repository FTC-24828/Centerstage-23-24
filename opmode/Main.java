package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.telecommand.ArmAdjustCommand;
import org.firstinspires.ftc.teamcode.commands.telecommand.ClawToggleCommand;
import org.firstinspires.ftc.teamcode.commands.telecommand.IntakeSequence;
import org.firstinspires.ftc.teamcode.commands.telecommand.DepositSequence;
import org.firstinspires.ftc.teamcode.commands.telecommand.IntermediateSequence;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.Global;
import org.firstinspires.ftc.teamcode.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Transformations;
import org.firstinspires.ftc.teamcode.util.Vector2D;

@TeleOp (name = "MainTeleOp")
public class Main extends CommandOpMode {
    private final Robot robot = Robot.getInstance();

    private GamepadEx controller;

    private double loop_time = 0.0;

    public double INITIAL_YAW;         //TODO LINK BETWEEN THE TWO PROGRAMS

    @Override
    public void initialize() {
        super.reset();

        Global.IS_AUTO = true;
        Global.USING_DASHBOARD = false;

        robot.addSubsystem(new Drivetrain(), new Intake(), new Arm());
        robot.init(hardwareMap, telemetry);

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
                .whenPressed(new IntakeSequence())
                .whenReleased(new IntermediateSequence());

        controller.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ConditionalCommand(
                        new IntermediateSequence(),
                        new DepositSequence(),
                        () -> Global.IS_SCORING
                ));


        while (opModeInInit()) {
            telemetry.addLine("Initialization complete.");
            telemetry.update();
        }
    }


    @Override
    public void run() {
        robot.read();

        Vector2D local_vector = Transformations.localOrientation(controller.getLeftX(), controller.getLeftY(), robot.getYaw());
        robot.drivetrain.move(local_vector, controller.getRightX());        //TODO ADD SHIFT MODE

        //LEFT TRIGGER GETS PRECEDENT
        if (controller.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5) {
            CommandScheduler.getInstance().schedule(new ArmAdjustCommand(-1));
        }
        else if (controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
            CommandScheduler.getInstance().schedule(new ArmAdjustCommand(1));
        }

        super.run();
        robot.periodic();

        double loop = System.nanoTime();
        telemetry.addData("Frequency", 1000000000 / (loop - loop_time) + "hz");
        telemetry.addData("Voltage", robot.getVoltage());
        telemetry.addData("arm angle", Math.toDegrees(robot.arm.arm_angle.getAsDouble()));
        telemetry.addData("wrist angle", Math.toDegrees(robot.intake.wrist_angle.getAsDouble()));
        telemetry.update();

        loop_time = loop;
        robot.write();
        robot.clearBulkCache(Global.Hub.EXPANSION_HUB);
    }
}