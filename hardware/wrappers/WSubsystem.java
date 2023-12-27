package org.firstinspires.ftc.teamcode.hardware.wrappers;

import com.arcrobotics.ftclib.command.Subsystem;

public abstract class WSubsystem {
    public abstract void periodic();
    public abstract void read();
    public abstract void write();
    public abstract void reset();
}
