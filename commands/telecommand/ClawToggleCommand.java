package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;

public class ClawToggleCommand extends ConditionalCommand {
    public ClawToggleCommand (Intake.ClawSide side) {
        super (
            new ClawCommand(side, Intake.ClawState.OPEN),
            new ClawCommand(side, Intake.ClawState.CLOSED),
                () -> Robot.getInstance().intake.getClawState(side) == Intake.ClawState.CLOSED
        );
    }
}
