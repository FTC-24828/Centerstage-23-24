package org.firstinspires.ftc.teamcode.commands.autocommand;


import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;
import org.firstinspires.ftc.teamcode.common.controllers.PIDF;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.drive.Drivetrain;
import org.firstinspires.ftc.teamcode.common.hardware.drive.pathing.Localizer;
import org.firstinspires.ftc.teamcode.common.util.Pose;

public class PositionCommand extends CommandBase {
    private final WRobot robot = WRobot.getInstance();

    private final Drivetrain drivetrain = robot.drivetrain;
    private final Localizer localizer = robot.localizer;
    private final Pose target_pose;

    public static double xP = 0.105;
    public static double xD = 0.0175;

    public static double yP = 0.105;
    public static double yD = 0.0175;

    public static double zP = 1.5;
    public static double zD = 0.075;

    public static PIDF xController = new PIDF(xP, 0.0, xD);
    public static PIDF yController = new PIDF(yP, 0.0, yD);
    public static PIDF zController = new PIDF(zP, 0.0, zD);

    public static double TRANSLATIONAL_TOLERANCE = 2;
    public static double YAW_TOLERANCE = 0.03;

    private ElapsedTime timer;
    private ElapsedTime stable;

    public static double DEAD_MS = 5000;
    public static double STABLE_MS = 200;

    public PositionCommand(Pose pose) {
        target_pose = pose;
    }

    public void execute() {
        if (timer == null) timer = new ElapsedTime();
        if (stable == null) stable = new ElapsedTime();

        Pose robot_pose = localizer.getPose();

        Pose powers = getPower(robot_pose);
        drivetrain.move(powers);
    }

    @Override
    public boolean isFinished() {
        Pose robot_pose = localizer.getPose();
        Pose delta = target_pose.subtract(robot_pose);

        if (delta.toVector2D().magnitude() > TRANSLATIONAL_TOLERANCE
                || Math.abs(delta.z) > YAW_TOLERANCE) {
            stable.reset();
        }

        return timer.milliseconds() > DEAD_MS || stable.milliseconds() > STABLE_MS;
    }

    public Pose getPower(Pose robot_pose) {
        if(target_pose.z - robot_pose.z > Math.PI) target_pose.z -= 2 * Math.PI;
        if(target_pose.z - robot_pose.z < -Math.PI) target_pose.z += 2 * Math.PI;

        double xPower = xController.calculate(robot_pose.x, target_pose.x);
        double yPower = yController.calculate(robot_pose.y, target_pose.y);
        double hPower = -zController.calculate(robot_pose.z, target_pose.z);

        double x_rotated = xPower * Math.cos(-robot_pose.z) - yPower * Math.sin(-robot_pose.z);
        double y_rotated = xPower * Math.sin(-robot_pose.z) + yPower * Math.cos(-robot_pose.z);

        hPower = Range.clip(hPower, -0.7, 0.7);

        return new Pose(x_rotated * 1.41, y_rotated, hPower);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.move(new Pose());
    }
}
