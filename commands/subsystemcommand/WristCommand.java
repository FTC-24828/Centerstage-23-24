package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;

public class WristCommand extends InstantCommand {
    public WristCommand(Intake.WristState state) {
        super (
                () -> Robot.getInstance().intake.setWristState(state)
        );
    }
}
