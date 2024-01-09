package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class ArmResetPosition extends InstantCommand {
    public ArmResetPosition() {
        super (
                () -> new ArmSetTargetCommand(WRobot.getInstance().arm_actuator.getReadingOffset())
        );
    }
}
