package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.autocommand.FirstStackSetup;
import org.firstinspires.ftc.teamcode.commands.autocommand.PositionCommand;
import org.firstinspires.ftc.teamcode.commands.autocommand.PurplePixelSequence;
import org.firstinspires.ftc.teamcode.commands.autocommand.YellowPixelSequence;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmResetPosition;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.util.Pose;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Blue Auto")
public class BlueAuto extends CommandOpMode {

    private final WRobot robot = WRobot.getInstance();

    private double loop_time = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double end_time = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Global.IS_AUTO = true;
        Global.USING_IMU = false;
        Global.USING_DASHBOARD = true;
        Global.USING_WEBCAM = true;
        Global.DEBUG = false;
        Global.SIDE = Global.Side.BLUE;

        robot.addSubsystem(new Drivetrain(), new Arm(), new Intake());
        robot.init(hardwareMap, telemetry);

        robot.arm.setArmState(Arm.ArmState.FLAT);
        robot.intake.setClawState(Intake.ClawSide.BOTH, Intake.ClawState.CLOSED);

        if (Global.USING_DASHBOARD) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            FtcDashboard.getInstance().startCameraStream(robot.pipeline, 0);
        }

        robot.localizer.reset(new Pose(0, 0, Math.PI)); //ANY WAY TO OFFSET ANGLE?

        robot.read();

        while (robot.vision_portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Autonomous initializing...");
            telemetry.update();
        }

        while (!isStarted()) {
            telemetry.addData("Path:", robot.pipeline.getPropLocation());
            telemetry.addLine("Ready");
            telemetry.update();
        }

        robot.resetYaw();

        Pose purple_pose = new Pose();
        Pose yellow_pose = new Pose();

        switch (robot.pipeline.getPropLocation()) {
            case LEFT:
                purple_pose = new Pose(-21, 25, -Math.PI / 2);
                yellow_pose = new Pose(-28, 25, -Math.PI / 2);
                break;
            case CENTER:
                purple_pose = new Pose(-14, 37, -Math.PI / 2);
                yellow_pose = new Pose(-28, 28.5, -Math.PI / 2);
                break;
            default:
                purple_pose = new Pose(2, 28, -Math.PI / 2);
                yellow_pose = new Pose(-28, 37, -Math.PI / 2);
                break;
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

                        //purple deposit
                        new PositionCommand(purple_pose)
                                .andThen(new PurplePixelSequence(robot)),

                        //yellow deposit
                        new PositionCommand(yellow_pose)
                                .andThen(new YellowPixelSequence(robot)),

                        //go to first stack
                        new PositionCommand(new Pose(-20, 47, -Math.PI / 2), 2000),
                        new PositionCommand(new Pose(75, 57, -Math.PI / 2)),
//                                .alongWith(new FirstStackSetup()),
//
//                        new FirstStackGrabCommand(),

//                        new PositionCommand(new Pose(-32, 3, 0)),

                        new InstantCommand(() -> end_time = timer.seconds())

                )
        );

        robot.vision_portal.setProcessorEnabled(robot.pipeline, false); //deallocate cpu resources
        robot.vision_portal.close();
    }


    @Override
    public void run() {
        robot.read();
        super.run();
        robot.periodic();
        robot.write();
        robot.clearBulkCache(Global.Hub.BOTH);

        double loop = System.nanoTime();
        telemetry.addData("Frequency", "%4.2fhz", 1000000000 / (loop - loop_time));
        telemetry.addData("Voltage", robot.getVoltage());
        telemetry.addData("Pose", robot.localizer.getPose().toString());
        telemetry.addData("Runtime:", "%.2f", end_time == 0 ? timer.seconds() : end_time);
        loop_time = loop;
        telemetry.update();
    }

    @Override
    public void reset() {
        super.reset();
        robot.reset();
        Global.resetGlobals();
//        robot.startIMUThread();
//        try {
//            robot.imu_thread.join();
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
        Global.YAW_OFFSET = robot.getYaw();
    }
}
