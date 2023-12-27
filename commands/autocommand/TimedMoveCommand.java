package org.firstinspires.ftc.teamcode.commands.autocommand;

import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class TimedMoveCommand extends ParallelDeadlineGroup {
    public TimedMoveCommand(double x, double y, double z, int milliseconds) {
        super(
                new WaitCommand(milliseconds),
                new RunCommand(() -> Robot.getInstance().drivetrain.move(x, y, z))
        );
    }
}
