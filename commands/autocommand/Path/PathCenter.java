package org.firstinspires.ftc.teamcode.commands.autocommand.Path;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.autocommand.PositionCommand;
import org.firstinspires.ftc.teamcode.commands.autocommand.PurplePixelSequence;
import org.firstinspires.ftc.teamcode.commands.autocommand.YellowPixelSequence;
import org.firstinspires.ftc.teamcode.common.util.Pose;

public class PathCenter extends SequentialCommandGroup {
    public PathCenter() {
        super(
                //purple deposit
                new PositionCommand(new Pose(-16, 40, -Math.PI / 2))
                        .andThen(new PurplePixelSequence()),

                //yellow deposit
                new PositionCommand(new Pose(-33, 26.5, -Math.PI / 2))
                        .andThen(new YellowPixelSequence())
        );
    }
}
