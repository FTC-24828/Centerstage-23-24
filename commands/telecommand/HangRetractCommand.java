package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.hang.SetHangPowerCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.hang.SetHangState;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Hang;

public class HangRetractCommand extends SequentialCommandGroup {
    public HangRetractCommand() {
        super(
                new SetHangState(Hang.HangState.RETRACT),
                new SetHangPowerCommand(-1)
        );
    }
}
