package org.firstinspires.ftc.teamcode.common.util;


import androidx.core.math.MathUtils;

public class WMath {
    public static double clamp(double value, double min, double max) {
        if (max < min) throw new IllegalArgumentException("tried to call WMath.clamp with illegal arguments");
        return Math.min(max, Math.max(value, min));
    }




}
