
package org.firstinspires.ftc.teamcode;

public class RunTime {
    interface RunTimeMethods {
        void move(double x, double y);   //move without turning, xy are left_stick values
        void turn(double x);     //pivot along center, xy are right_stick values
        void apDetection();
        void tfDetection();
    }
}