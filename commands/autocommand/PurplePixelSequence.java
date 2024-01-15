package org.firstinspires.ftc.teamcode.commands.autocommand;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class PurplePixelSequence extends SequentialCommandGroup {
    public PurplePixelSequence() {
        super (
                new WristCommand(Intake.WristState.FLAT),
                new WaitCommand(500),
                new ClawCommand(Intake.ClawSide.RIGHT, Intake.ClawState.OPEN),
                new WaitCommand(200),
                new WristCommand(Intake.WristState.FOLD),
                new WaitCommand(100)
            );
    }
}
