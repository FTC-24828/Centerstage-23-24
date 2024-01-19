package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.hang.SetHangPowerCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.hang.SetHangState;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Hang;

public class HangStopCommand extends SequentialCommandGroup {
    public HangStopCommand() {
        super(
                new SetHangState(Hang.HangState.STOPPED),
//                new SetHangPowerCommand(0),
                new InstantCommand(() -> WRobot.getInstance().hang.setPowerMotor(0))

        );
    }
}
