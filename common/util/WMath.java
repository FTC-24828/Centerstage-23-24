package org.firstinspires.ftc.teamcode.common.util;


public class WMath {
    public static double clamp(double value, double min, double max) {
        if (max < min) throw new IllegalArgumentException("tried to call WMath.clamp with illegal arguments");
        return Math.min(max, Math.max(value, min));
    }

    /**Wraps angle and return a coterminal angles between pi to, but not including, -pi*/
    public static double wrapAngle(double theta) {
        final double twoPI = 2 * Math.PI;
        theta %= twoPI;
        theta = (theta + twoPI) % twoPI;
        if (theta > Math.PI) theta -= twoPI;
        return theta;
    }

//    public static double wrapShort(double current, double target) {
//        return Math.min(target - current)
//    }

    public static double max(double a, double b, double c) {
        return Math.max(Math.max(a, b), c);
    }
}