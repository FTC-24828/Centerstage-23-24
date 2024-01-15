package org.firstinspires.ftc.teamcode.commands.autocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class FirstStackGrabCommand extends SequentialCommandGroup {
    public FirstStackGrabCommand() {
        super(
                new ClawCommand(Intake.ClawSide.RIGHT, Intake.ClawState.CLOSED)
        );
    }
}
