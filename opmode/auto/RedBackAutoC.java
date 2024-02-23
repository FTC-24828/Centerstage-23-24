package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.autocommand.PositionCommand;
import org.firstinspires.ftc.teamcode.commands.autocommand.PurplePixelSequence;
import org.firstinspires.ftc.teamcode.commands.autocommand.YellowPixelSequence;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Pose;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Back AutoC")
public class RedBackAutoC extends CommandOpMode {

    private final WRobot robot = WRobot.getInstance();

    private double loop_time = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double end_time = 0;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        Global.IS_AUTO = true;
        Global.USING_IMU = true;
        Global.USING_DASHBOARD = false;
        Global.USING_WEBCAM = false;
        Global.DEBUG = true;
        Global.SIDE = Global.Side.RED;

        robot.addSubsystem(new Drivetrain(), new Arm(), new Intake());
        robot.init(hardwareMap, telemetry);

        robot.arm.setArmState(Arm.ArmState.FLAT);
        robot.intake.setWristState(Intake.WristState.FOLD);
        robot.intake.setClawState(Intake.ClawSide.BOTH, Intake.ClawState.CLOSED);

        if (Global.USING_DASHBOARD) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            FtcDashboard.getInstance().startCameraStream(robot.pipeline, 0);
        }

        robot.localizer.reset(new Pose(0, 0, -Math.PI));
        robot.read();

//        if (Global.USING_WEBCAM) {
//            while (robot.vision_portal.getCameraState() != VisionPortal.CameraState.STREAMING && robot.pipeline.getPropLocation() == null) {
//                telemetry.addLine("Autonomous initializing...");
//                telemetry.update();
//            }


        while (!isStarted()) {
//            telemetry.addData("Path:", robot.pipeline.getPropLocation());
            telemetry.addData("Pose", robot.localizer.getPose().toString());
            telemetry.addData("Encoder readings", "%.2f, %.2f, %.2f",
                    robot.encoder_readings.get(Sensors.Encoder.POD_LEFT),
                    robot.encoder_readings.get(Sensors.Encoder.POD_MIDDLE),
                    robot.encoder_readings.get(Sensors.Encoder.POD_RIGHT));
            telemetry.addLine("Ready");
            telemetry.update();
        }

        robot.resetYaw();

        Pose purple_pose;
        Pose yellow_pose;
        Pose left_spike;
        Pose repose1;
        Pose repose2;

        Global.PropLocation chosen = Global.PropLocation.CENTER;

        switch (chosen) {
            case LEFT:
                purple_pose = new Pose(4.5, 28, Math.PI / 2);
                left_spike = new Pose(-1.5, 28, Math.PI / 2);
                repose1 = new Pose( .33, 2.19, 1.44);
                repose2 = new Pose( 55.70, .45, 1.49);
                yellow_pose = new Pose(109.5, 30, Math.PI / 2);
                break;
            case  CENTER:
                purple_pose = new Pose(-12, 38, -Math.PI/2);
                left_spike = purple_pose;
                repose1 = new Pose(-12, 2.5, -Math.PI/2);
                repose2 = new Pose(55, 3.5, -Math.PI/2);
                yellow_pose = new Pose(80, 32, Math.PI / 2);
                break;
            default:
                purple_pose = new Pose(2.5, 26, -Math.PI/2);
                left_spike = purple_pose;
                repose1 = new Pose( .33, 2.5, -Math.PI/2);
                repose2 = new Pose( 55.70, 3, -Math.PI/2);
                yellow_pose = new Pose(78, 24, Math.PI/2);

                break;
        }

//        Pose first_stack_pose = new Pose(79, 58, -Math.PI / 2);
//        Pose first_stack_deposit = new Pose(-29, 31.5, -Math.PI / 2);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

                        //purple deposit
                        new PositionCommand(purple_pose)
                                .andThen(new PositionCommand(left_spike))
                                .andThen(new PurplePixelSequence()),

                        //reposition
                        new PositionCommand(repose1)
                                .andThen(new PositionCommand(repose2)),

                        //yellow deposit
                        new PositionCommand(yellow_pose)
                                .andThen(new YellowPixelSequence()),

//                        //go to first stack
//                        new PositionCommand(new Pose(-20, 47, -Math.PI / 2)),
//                        new PositionCommand(first_stack_pose)
//                                .alongWith(new FirstStackSetup()),

//                        new FirstStackGrabCommand(),
//                        new WaitCommand(500),
//
//                        new PositionCommand(new Pose(-20, 60, -Math.PI / 2)),
//
//                        new PositionCommand(first_stack_deposit)
//                                .andThen(new FirstStackDeposit()),

                        new InstantCommand(() -> end_time = timer.seconds())

                )
        );

        if (Global.USING_WEBCAM) {
//            robot.vision_portal.setProcessorEnabled(robot.pipeline, false); //deallocate cpu resources
//            robot.vision_portal.close();
        }
    }


    @Override
    public void run() {
        robot.read();
        super.run();
        robot.periodic();
        robot.write();
        robot.clearBulkCache(Global.Hub.CONTROL_HUB);

        double loop = System.nanoTime();
        telemetry.addData("Frequency", "%3.2fhz", 1000000000 / (loop - loop_time));
        telemetry.addData("Voltage", robot.getVoltage());
        telemetry.addData("Pose", robot.localizer.getPose().toString());
        telemetry.addData("Runtime:", "%.2f", end_time == 0 ? timer.seconds() : end_time);

        if (Global.DEBUG) {
            telemetry.addData("Encoder readings", "%.2f, %.2f, %.2f",
                    robot.encoder_readings.get(Sensors.Encoder.POD_LEFT),
                    robot.encoder_readings.get(Sensors.Encoder.POD_MIDDLE),
                    robot.encoder_readings.get(Sensors.Encoder.POD_RIGHT));

            telemetry.addLine("---------------------------");
            telemetry.addData("arm target", robot.arm.target_position);
            telemetry.addData("arm power", robot.arm.power);
            telemetry.addData("arm state", robot.arm.getArmState());

            telemetry.addLine("---------------------------");
            telemetry.addData("wrist target", robot.intake.target_position);
            telemetry.addData("wrist angle", robot.intake.wrist_angle);
        }

        loop_time = loop;
        telemetry.update();
    }

    @Override
    public void reset() {
        super.reset();
        robot.reset();
        Global.resetGlobals();
//        robot.updateYaw();
        Global.YAW_OFFSET = robot.getYaw();
    }
}
