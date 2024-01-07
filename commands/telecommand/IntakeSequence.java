package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmSetStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class IntakeSequence extends SequentialCommandGroup {
    public IntakeSequence() {
        super(
            new InstantCommand(() -> Global.setState(Global.State.INTAKING)),
            new ArmSetStateCommand(Arm.ArmState.FLAT),
            new WristCommand(Intake.WristState.FLAT),
            new WaitCommand(200),
            new ClawCommand(Intake.ClawSide.BOTH, Intake.ClawState.OPEN)
        );
    }
}
