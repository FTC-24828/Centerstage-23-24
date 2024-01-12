package org.firstinspires.ftc.teamcode.commands.autocommand.Path;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.autocommand.PositionCommand;
import org.firstinspires.ftc.teamcode.commands.autocommand.PurplePixelSequence;
import org.firstinspires.ftc.teamcode.commands.autocommand.YellowPixelSequence;
import org.firstinspires.ftc.teamcode.common.util.Pose;

public class PathLeft extends SequentialCommandGroup {
    public PathLeft() {
        super(
                //purple deposit
                new PositionCommand(new Pose(-22.5, 27, -Math.PI / 2))
                        .andThen(new PurplePixelSequence()),

                //yellow deposit
                new PositionCommand(new Pose(-32, 26.33, -Math.PI / 2))
                        .andThen(new YellowPixelSequence())
        );
    }
}
