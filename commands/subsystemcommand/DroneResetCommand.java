package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;

public class DroneResetCommand extends InstantCommand {
    public DroneResetCommand(WRobot robot) {
        super (
                robot.drone::resetDrone
        );
    }
}
