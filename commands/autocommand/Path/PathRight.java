package org.firstinspires.ftc.teamcode.commands.autocommand.Path;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.autocommand.PositionCommand;
import org.firstinspires.ftc.teamcode.commands.autocommand.PurplePixelSequence;
import org.firstinspires.ftc.teamcode.commands.autocommand.YellowPixelSequence;
import org.firstinspires.ftc.teamcode.common.util.Pose;

public class PathRight extends SequentialCommandGroup {
    public PathRight() {
        super(
                //purple deposit
                new PositionCommand(new Pose(3, 29, -Math.PI / 2))
                        .andThen(new PurplePixelSequence()),

                //yellow deposit
                new PositionCommand(new Pose(-32, 40, -Math.PI / 2))
                        .andThen(new YellowPixelSequence())
        );
    }
}
