package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

@Config
@Autonomous(name = "Red Auto")
public class RedAuto extends CommandOpMode {

    private final WRobot robot  = WRobot.getInstance();


    private double loop_time = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double end_time = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Global.IS_AUTO = true;
        Global.USING_IMU = true;
        Global.USING_DASHBOARD = true;
        Global.COLOR = Global.Side.RED;

        robot.addSubsystem(new Drivetrain(), new Intake(), new Arm());
        robot.init(hardwareMap, telemetry);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.intake.setClawState(Intake.ClawSide.BOTH, Intake.ClawState.CLOSED);

        robot.read();
        while (!isStarted()) {
            telemetry.addLine("Autonomous initializing...");
            telemetry.update();
        }

//        robot.localizer.setPoseEstimate(new Pose2d(0, 0, 0));


        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),
                        // go to yellow pixel scoring pos
//                        new PositionCommand(new Pose(25, 0, 0))
//                                .alongWith(new PurplePixelExtendCommand()),
//
//                        new PurplePixelDepositCommand(),
//
//                        new PositionCommand(new Pose(25, -0.25, -Math.PI / 2))
//                                .alongWith(new FirstStackSetupCommand()),
//
//                        new FirstStackGrabCommand(),
//
//                        new PositionCommand(new Pose(27, -68.25, -Math.PI / 2))
//                                .alongWith(new AutoDepositCommand()),

                        new InstantCommand(() -> end_time = timer.seconds())

                )
        );
    }

    @Override
    public void run() {
        robot.read();
        super.run();
        robot.periodic();

        double loop = System.nanoTime();
        telemetry.addData("hz ", 1000000000 / (loop - loop_time));
//        telemetry.addLine(robot.localizer.getPos().toString());
        telemetry.addData("Runtime: ", end_time == 0 ? timer.seconds() : end_time);
        loop_time = loop;
        telemetry.update();

        robot.write();
        robot.clearBulkCache(Global.Hub.BOTH);
    }
}
