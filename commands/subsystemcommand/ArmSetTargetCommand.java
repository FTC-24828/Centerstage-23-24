package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class ArmSetTargetCommand extends InstantCommand {
    public ArmSetTargetCommand(WRobot robot, double target) {
        super(
                () -> robot.arm.setTargetPosition(target)
        );
    }
}
