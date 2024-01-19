package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.hang.SetHangPowerCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.hang.SetHangState;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Hang;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class HangCommand extends SequentialCommandGroup {
    public HangCommand() {
        super(
                new SetHangState(Hang.HangState.HANGING),
//                new SetHangPowerCommand(1),
                new InstantCommand(() -> WRobot.getInstance().hang.setPowerMotor(1))
        );
    }
}
