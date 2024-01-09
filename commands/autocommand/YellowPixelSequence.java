package org.firstinspires.ftc.teamcode.commands.autocommand;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.subsystemcommand.ArmSetTargetCommand;
import org.firstinspires.ftc.teamcode.commands.subsystemcommand.WristCommand;
import org.firstinspires.ftc.teamcode.commands.telecommand.ArmAdjustCommand;
import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.subsystems.Intake;

public class YellowPixelSequence extends SequentialCommandGroup {
    public YellowPixelSequence() {
        super(
                new WaitCommand(500),
                new ArmSetTargetCommand((double) 5 * Global.TETRIX_MOTOR_TPR / 4)
                        .alongWith(new WristCommand(Intake.WristState.SCORING))
        );
    }
}
