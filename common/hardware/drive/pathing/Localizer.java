package org.firstinspires.ftc.teamcode.common.hardware.drive.pathing;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.util.Vector2D;
import org.firstinspires.ftc.teamcode.common.util.WMath;

import java.util.function.DoubleSupplier;

@Config
public class Localizer {
    private final WRobot robot = WRobot.getInstance();

    private Pose start;
    private Pose pose;

    public static double WHEEL_RADIUS = 3.96;
    public static double WHEEL_TOE = 0.7784;
    public static double TRACK_WIDTH = 6.67;

    private DoubleSupplier p_tr, p_tl, p_br, p_bl;
    private double _tr, _tl, _br, _bl;

    public double d_tr;
    public double d_tl;
    public double d_br;
    public double d_bl;

    double local_dx, local_dy;

    public Localizer(Pose pose) {
        start = pose;
        this.pose = start;
    }

    public void init() {
        p_tr = () -> robot.doubleSubscriber(Sensors.Encoder.RIGHT_FRONT);
        p_tl = () -> robot.doubleSubscriber(Sensors.Encoder.LEFT_FRONT);
        p_br = () -> robot.doubleSubscriber(Sensors.Encoder.RIGHT_REAR);
        p_bl = () -> robot.doubleSubscriber(Sensors.Encoder.LEFT_REAR);
        read();
    }

    public void read() {
        _tr = p_tr.getAsDouble();
        _tl = p_tl.getAsDouble();
        _br = p_br.getAsDouble();
        _bl = p_bl.getAsDouble();
    }

    public void update() {
         d_tr = ticksToInches(p_tr.getAsDouble() - _tr);
         d_tl = ticksToInches(p_tl.getAsDouble() - _tl);
         d_br = ticksToInches(p_br.getAsDouble() - _br);
         d_bl = ticksToInches(p_bl.getAsDouble() - _bl);

        this.pose.z = WMath.wrapAngle(pose.z + (d_tr + d_tl - d_br - d_bl) / (4.0 * Math.PI * TRACK_WIDTH));
        local_dx = Math.sin(WHEEL_TOE) * (d_tr - d_tl + d_br - d_bl) / 4.0;
        local_dy = Math.cos(WHEEL_TOE) * (d_tr + d_tl + d_br + d_bl) / 4.0;
        Vector2D translated = new Vector2D(local_dx, local_dy, -pose.z);
        pose.x += translated.x;
        pose.y += translated.y;

        read();
    }

    private double ticksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / Global.TETRIX_MOTOR_TPR;
    }

    public void reset(Pose p) {
        setPose(p);
        setStart(p);
    }

    public void reset() {
        reset(new Pose());
    }

    public void setStart(Pose pose) {
        start = pose;
    }

    public void setPose(Pose p) {
        this.pose = p;
    }

    public Pose getPose() {
        return pose;
    }
}
