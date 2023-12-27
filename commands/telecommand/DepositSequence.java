package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.hardware.Global;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;

public class DepositSequence extends SequentialCommandGroup {
    public DepositSequence() {
        super(
                new InstantCommand(Global::startScoring),
                new ArmCommand(Arm.ArmState.SCORING),
                new WristCommand(Intake.WristState.SCORING)
        );
    }
}
