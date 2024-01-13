package org.firstinspires.ftc.teamcode.commands.telecommand;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmSetStateCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Arm;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class DepositSequence extends ParallelCommandGroup {
    public DepositSequence(WRobot robot) {
        super(
                new InstantCommand(() -> Global.setState(Global.State.SCORING)),
                new ArmSetStateCommand(robot, Arm.ArmState.SCORING),
                new SequentialCommandGroup(
//                        new WristCommand(robot, Intake.WristState.FLAT),
//                        new WaitCommand(1000),
                        new WristCommand(robot, Intake.WristState.SCORING)
                )
        );
    }
}
