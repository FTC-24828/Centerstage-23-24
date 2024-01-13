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
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.vision.PropPipeline;

@Config
@Autonomous(name = "Red Auto")
public class RedAuto extends CommandOpMode {

    private final WRobot robot  = WRobot.getInstance();

    private PropPipeline pipeline;

    private double loop_time = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double end_time = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Global.IS_AUTO = true;
        Global.USING_IMU = true;
        Global.USING_DASHBOARD = true;
        Global.SIDE = Global.Side.RED;

        robot.addSubsystem(new Drivetrain(), new Intake(), new Arm());
        robot.init(hardwareMap, telemetry);

        if (Global.USING_DASHBOARD) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            FtcDashboard.getInstance().startCameraStream(pipeline, 0);
        }
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

    @Override
    public void reset() {
        super.reset();
        robot.reset();
        Global.resetGlobals();
    }
}
