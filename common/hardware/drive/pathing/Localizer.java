package org.firstinspires.ftc.teamcode.common.hardware.drive.pathing;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.hardware.Global;
import org.firstinspires.ftc.teamcode.common.hardware.Sensors;
import org.firstinspires.ftc.teamcode.common.hardware.WRobot;
import org.firstinspires.ftc.teamcode.common.util.Pose;
import org.firstinspires.ftc.teamcode.common.util.WMath;

import java.util.function.DoubleSupplier;

@Config
public class Localizer {
    private final WRobot robot = WRobot.getInstance();

    private Pose start;
    private Pose pose;

    public static double WHEEL_RADIUS = 7.62;
    public static double WHEEL_TOE = 0.698131;
    public static double TRACK_WIDTH = 43.18;

    private DoubleSupplier p_tr, p_tl, p_br, p_bl;
    private double _tr, _tl, _br, _bl;

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
        double d_tr = ticksToCm(p_tr.getAsDouble() - _tr);
        double d_tl = ticksToCm(p_tl.getAsDouble() - _tl);
        double d_br = ticksToCm(p_br.getAsDouble() - _br);
        double d_bl = ticksToCm(p_bl.getAsDouble() - _bl);

        this.pose.x += Math.cos(WHEEL_TOE) * (d_tr + d_tl + d_br + d_bl) / 4.0;
        this.pose.y += Math.sin(WHEEL_TOE) * (d_tr - d_tl - d_br + d_bl) / 4.0;
        this.pose.z = WMath.wrapAngle(pose.z + (d_tr - d_tl + d_br - d_bl) / (8.0 * Math.PI * TRACK_WIDTH));
        read();
    }

    private double ticksToCm(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / Global.TETRIX_MOTOR_TPR;
    }

    public void reset() {
        setPose(new Pose());
        setStart(new Pose());
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
