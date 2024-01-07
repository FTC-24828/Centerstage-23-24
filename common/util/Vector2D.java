package org.firstinspires.ftc.teamcode.common.util;

public class Vector2D {
        public double x, y;

        public Vector2D(double x, double y) {
                this.x = x; this.y = y;
        }

        /**returns x and y values relative to the robot offset by a forward z angle*/
        public Vector2D(double x, double y, double z) {
                this.x = x * Math.cos(z) + y * Math.sin(z);
                this.y = x * -Math.sin(z) + y * Math.cos(z);
        }

        public double magnitude() {
                return Math.hypot(x, y);
        }

        public void scale(double scalar) {
                this.x *= scalar;
                this.y *= scalar;
        }

}
