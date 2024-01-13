package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class ClawCommand extends InstantCommand {
    public ClawCommand (WRobot robot, Intake.ClawSide side, Intake.ClawState state) {
        super(
                () -> robot.intake.setClawState(side, state)
        );
    }
}
