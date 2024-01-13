package org.firstinspires.ftc.teamcode.commands.autocommand;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class PurplePixelSequence extends SequentialCommandGroup {
    public PurplePixelSequence(WRobot robot) {
        super (
                new WristCommand(robot, Intake.WristState.FLAT),
                new WaitCommand(500),
                new ClawCommand(robot, Intake.ClawSide.RIGHT, Intake.ClawState.OPEN),
                new WaitCommand(200),
                new WristCommand(robot, Intake.WristState.FOLD),
                new WaitCommand(200)
            );
    }
}
