package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class WristCommand extends InstantCommand {
    public WristCommand(WRobot robot, Intake.WristState state) {
        super (
                () -> robot.intake.setWristState(state)
        );
    }
}
