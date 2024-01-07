package org.firstinspires.ftc.teamcode.common.util;


public class WMath {
    public static double clamp(double value, double min, double max) {
        if (max < min) throw new IllegalArgumentException("tried to call WMath.clamp with illegal arguments");
        return Math.min(max, Math.max(value, min));
    }

    /**Wraps angle and return a coterminal angles between 0 to, but not including, 2pi*/
    public static double wrapAngle(double theta) {
        final double twoPI = 2 * Math.PI;
        theta %= twoPI;
        return (theta + twoPI) % twoPI;
    }

    public static double max(double a, double b, double c) {
        return Math.max(Math.max(a, b), c);
    }
}
