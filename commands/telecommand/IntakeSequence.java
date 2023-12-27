package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;

public class IntakeSequence extends SequentialCommandGroup {
    public IntakeSequence() {
        super(
            new ArmCommand(Arm.ArmState.FLAT),
            new WristCommand(Intake.WristState.FLAT),
            new WaitCommand(100),
            new ClawCommand(Intake.ClawSide.BOTH, Intake.ClawState.OPEN)
        );
    }
}
