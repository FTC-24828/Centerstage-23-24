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
    public YellowPixelSequence() {
        super(
                new ArmSetStateCommand(Arm.ArmState.SCORING),
                new ArmSetTargetCommand((double) Global.TETRIX_MOTOR_TPR)
                        .alongWith(new WristCommand(Intake.WristState.SCORING)),

                 new ArmSetTargetCommand((double) 9 * Global.TETRIX_MOTOR_TPR / 6)
                         .alongWith(new WristSetIncrement(-0.3)),
                new WaitCommand(2000),

                new ClawCommand(Intake.ClawSide.LEFT, Intake.ClawState.OPEN),
                new WaitCommand(500),

                new ArmSetTargetCommand((double) 4 * Global.TETRIX_MOTOR_TPR / 3 - 400),
                new WaitCommand(1000),

                new ClawCommand(Intake.ClawSide.BOTH, Intake.ClawState.CLOSED),

                new ArmSetStateCommand( Arm.ArmState.FLAT)
                        .alongWith(new WristCommand(Intake.WristState.FOLD)),
                new WaitCommand(500)
        );
    }
}
