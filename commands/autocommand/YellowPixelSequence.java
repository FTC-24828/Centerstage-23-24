package org.firstinspires.ftc.teamcode.commands.autocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmSetStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmSetTargetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristSetIncrement;
import org.firstinspires.ftc.teamcode.commands.telecommand.ArmAdjustCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class YellowPixelSequence extends SequentialCommandGroup {
    public YellowPixelSequence(WRobot robot) {
        super(
                new ArmSetStateCommand(robot, Arm.ArmState.SCORING),
                new ArmSetTargetCommand(robot, (double) Global.TETRIX_MOTOR_TPR)
                        .alongWith(new WristCommand(robot, Intake.WristState.SCORING)),
//                        .alongWith(new WristCommand(robot, Intake.WristState.FLAT))
//                        .alongWith(new WristSetIncrement(robot, -1)),
                 new ArmSetTargetCommand(robot, (double) 3 * Global.TETRIX_MOTOR_TPR / 2),
                new WaitCommand(1000),
                new WristSetIncrement(robot, -0.3),
                new WaitCommand(1000),
                new ClawCommand(robot, Intake.ClawSide.LEFT, Intake.ClawState.OPEN),
                new WaitCommand(500),
                new ArmSetTargetCommand(robot, (double) 4 * Global.TETRIX_MOTOR_TPR / 3 - 400),
                new WaitCommand(1000),
                new ClawCommand(robot, Intake.ClawSide.BOTH, Intake.ClawState.CLOSED),
                new WristSetIncrement(robot, 0),
                new ArmSetStateCommand(robot, Arm.ArmState.FLAT)
                        .alongWith(new WristCommand(robot, Intake.WristState.FOLD)),
                new WaitCommand(500)
        );
    }
}
