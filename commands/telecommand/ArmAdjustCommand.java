package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class ArmAdjustCommand extends InstantCommand {
    public ArmAdjustCommand(int increment) {
        super(
                () -> Robot.getInstance().arm.incrementHeight(increment)
        );
    }
}
