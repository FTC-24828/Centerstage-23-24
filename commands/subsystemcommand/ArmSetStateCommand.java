package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;

public class ArmSetStateCommand extends InstantCommand {
    public ArmSetStateCommand(WRobot robot, Arm.ArmState state) {
        super (
                () -> robot.arm.setArmState(state)
        );
    }
}
