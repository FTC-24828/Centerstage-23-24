package org.firstinspires.ftc.teamcode.commands.autocommand;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class TimedMoveCommand extends ParallelDeadlineGroup {
    /**
     * risky method for autonomous
     @param x          horizontal movement
     @param y           vertical movement
     @param z           local direction vector offset (keep 0)
     @param milliseconds      time in milliseconds
     */
    public TimedMoveCommand(double x, double y, double z, int milliseconds) {
        super(
                new WaitCommand(milliseconds),
                new RunCommand(() -> WRobot.getInstance().drivetrain.move(x, y, z))
        );
    }
}
