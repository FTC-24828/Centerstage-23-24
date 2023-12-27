package org.firstinspires.ftc.teamcode.util;

public class Transformations {

    //returns x and y values relative to robot using the yaw angle and movement joystick
    public static Vector2D localOrientation(double x, double y, double z) {
        return new Vector2D(
                -x * Math.sin(z + Math.PI/2) - y * Math.sin(z),
                x * Math.cos(z + Math.PI/2) + y * Math.cos(z)
        );
    }
}
