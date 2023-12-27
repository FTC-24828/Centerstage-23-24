package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;

public class ClawCommand extends InstantCommand {
    public ClawCommand (Intake.ClawSide side, Intake.ClawState state) {
        super(
                () -> Robot.getInstance().intake.setClawState(side, state)
        );
    }
}
