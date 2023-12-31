package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmSetStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.hardware.Global;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;

public class IntermediateSequence extends ParallelCommandGroup {
    public IntermediateSequence() {
        super(
                new InstantCommand(Global::stopScoring),
                new InstantCommand(Global::stopIntaking),
                new ArmSetStateCommand(Arm.ArmState.FLAT),
                new WristCommand(Intake.WristState.FOLD),
                new SequentialCommandGroup(
                        new WaitCommand(300),
                        new ClawCommand(Intake.ClawSide.BOTH, Intake.ClawState.CLOSED)
                )
        );
    }
}
