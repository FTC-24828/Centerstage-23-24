package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class WristSetIncrement extends InstantCommand {
    public WristSetIncrement(WRobot robot, double i) {
        super(
                () -> robot.intake.increment = i
        );
    }
}
