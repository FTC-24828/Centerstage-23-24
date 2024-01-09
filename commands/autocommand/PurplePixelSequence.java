package org.firstinspires.ftc.teamcode.commands.autocommand;


import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class PurplePixelSequence extends SequentialCommandGroup {
    public PurplePixelSequence() {
        super (
                new WaitCommand(300),
                new WristCommand(Intake.WristState.FLAT)
        );
    }
}