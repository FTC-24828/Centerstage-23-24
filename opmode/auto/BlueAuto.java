package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        Global.USING_WEBCAM = false;
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

        while (!isStarted()) {
            //telemetry.addData("Path:", robot.pipeline.getPropLocation());
            telemetry.addLine("Autonomous initializing...");
            telemetry.update();
        }

        robot.resetYaw();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new InstantCommand(timer::reset),

                        //purple deposit
                        new PositionCommand(new Pose(-22.5, 28, -Math.PI / 2))
                                .andThen(new PurplePixelSequence()),

                        //yellow deposit
                        new PositionCommand(new Pose(-30.5, 24.5, -Math.PI / 2))
                                .andThen(new YellowPixelSequence()),


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

//        robot.vision_portal.setProcessorEnabled(robot.pipeline, false); //deallocate cpu resources
//        robot.vision_portal.close();
    }


    @Override
    public void run() {
        robot.read();
        super.run();
        robot.periodic();

        double loop = System.nanoTime();
        telemetry.addData("Frequency", "%.2fhz", 1000000000 / (loop - loop_time));
        telemetry.addData("Voltage", robot.getVoltage());
        telemetry.addData("Pose", robot.localizer.getPose().toString());
        telemetry.addData("arm angle", "%.2f", Math.toDegrees(robot.arm.arm_angle.getAsDouble()));
        telemetry.addData("Runtime: ", end_time == 0 ? timer.seconds() : end_time);

//        telemetry.addLine("-------------------------------");
//        telemetry.addData("z err", "%.2f", PositionCommand.zController.last_error);
//        telemetry.addData("z pose", "%.2f", robot.localizer.getPose().z);
//        telemetry.addData("z output", "%.2f", PositionCommand.zController.current_output);
//        telemetry.addData("y output", "%.2f", PositionCommand.yController.current_output);
//        telemetry.addData("Baseline", 0);
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
//        robot.startIMUThread();
//        try {
//            robot.imu_thread.join();
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
        Global.YAW_OFFSET = robot.getYaw();
    }
}
