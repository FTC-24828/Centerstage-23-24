package org.firstinspires.ftc.teamcode.commands.autocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class YellowPixelDeposit extends SequentialCommandGroup {
    public YellowPixelDeposit() {
        super(
                new WaitCommand(100),
                new ClawCommand(Intake.ClawSide.LEFT, Intake.ClawState.OPEN)
        );
    }
}
