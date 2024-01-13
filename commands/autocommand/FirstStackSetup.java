package org.firstinspires.ftc.teamcode.commands.autocommand;

import android.graphics.Path;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmSetStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmSetTargetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristSetIncrement;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class FirstStackSetup extends SequentialCommandGroup {
    public FirstStackSetup(WRobot robot) {
        super (
                new WaitCommand(3000),
                new ArmSetStateCommand(robot, Arm.ArmState.SCORING),
                new ArmSetTargetCommand(robot, 0),
                new WaitCommand(500),
                new WristSetIncrement(robot, 0.2),
                new WristCommand(robot, Intake.WristState.FLAT),
                new ClawCommand(robot, Intake.ClawSide.RIGHT, Intake.ClawState.OPEN)
        );
    }
}
