package org.firstinspires.ftc.teamcode.commands.autocommand;

import android.graphics.Path;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmSetTargetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class FirstStackSetup extends SequentialCommandGroup {
    public FirstStackSetup() {
        super (
                new ArmSetTargetCommand(WRobot.getInstance().arm_actuator.getReadingOffset() + 70),
                new WaitCommand(200),
                new WristCommand(Intake.WristState.FLAT),
                new ClawCommand(Intake.ClawSide.LEFT, Intake.ClawState.OPEN)
        );
    }
}
