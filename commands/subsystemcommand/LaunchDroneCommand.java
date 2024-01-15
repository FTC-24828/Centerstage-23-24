package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class LaunchDroneCommand extends InstantCommand {
    public LaunchDroneCommand() {
        super (
                WRobot.getInstance().drone::launchDrone
        );
    }
}
