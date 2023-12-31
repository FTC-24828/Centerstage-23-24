package org.firstinspires.ftc.teamcode.util;

public class Vector2D {
        public double x, y;

        public Vector2D(double x, double y) {
                this.x = x; this.y = y;
        }


        //returns x and y values relative to robot using the yaw angle and movement joystick
        public static Vector2D toLocalOrientation(double x, double y, double z) {
                return new Vector2D(
                        -x * Math.sin(z + Math.PI / 2) - y * Math.sin(z),
                        x * Math.cos(z + Math.PI / 2) + y * Math.cos(z)
                );
        }

        public void scale(double value) {
                this.x *= value;
                this.y *= value;
        }

}
