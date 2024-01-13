package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmResetIncrementCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmSetStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ClawCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class IntermediateSequence extends ParallelCommandGroup {
    public IntermediateSequence(WRobot robot) {
        super(
                new InstantCommand(() -> Global.setState(Global.State.INTERMEDIATE)),
                new ArmSetStateCommand(robot, Arm.ArmState.FLAT),
                new ArmResetIncrementCommand(robot),
                new WristCommand(robot, Intake.WristState.FOLD),
                new SequentialCommandGroup(
                        new WaitCommand(300),
                        new ClawCommand(robot, Intake.ClawSide.BOTH, Intake.ClawState.CLOSED)
                )
        );
    }
}
