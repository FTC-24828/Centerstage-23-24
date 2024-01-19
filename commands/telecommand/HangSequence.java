package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.arm.ArmSetStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.hang.HookCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.intake.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class HangSequence extends SequentialCommandGroup {
    public HangSequence() {
        super(
                new InstantCommand(() -> Global.setState(Global.State.HANGING)),
                new WristCommand(Intake.WristState.LAUNCHING),
                new ArmSetStateCommand(Arm.ArmState.HANG),
                new WaitCommand(1000),
                new HookCommand(1)
        );
    }
}
