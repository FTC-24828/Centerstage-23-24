package org.firstinspires.ftc.teamcode.commands.subsystemcommand;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Arm;

public class ArmSetStateCommand extends InstantCommand {
    public ArmSetStateCommand(Arm.ArmState state) {
        super (
                () -> Robot.getInstance().arm.setArmState(state)
        );
    }
}
