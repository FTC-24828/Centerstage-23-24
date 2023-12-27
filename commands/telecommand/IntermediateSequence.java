package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.hardware.Global;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;

public class IntermediateSequence extends SequentialCommandGroup {
    public IntermediateSequence() {
        super(
                new InstantCommand(Global::stopScoring),
                new ArmCommand(Arm.ArmState.FLAT),
                new WristCommand(Intake.WristState.FOLD),
                new ClawCommand(Intake.ClawSide.BOTH, Intake.ClawState.CLOSED)
        );
    }
}
