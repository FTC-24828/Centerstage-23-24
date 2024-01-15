package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class WristPositionCommand extends InstantCommand {
    public WristPositionCommand(double position) {
        super(
                () -> WRobot.getInstance().wrist.setPosition(position)
        );
    }
}
