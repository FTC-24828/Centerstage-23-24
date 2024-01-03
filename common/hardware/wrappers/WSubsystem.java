package org.firstinspires.ftc.teamcode.common.hardware.wrappers;

import com.arcrobotics.ftclib.command.Subsystem;

public interface WSubsystem {
    void periodic();
    void read();
    void write();
    void reset();
}
