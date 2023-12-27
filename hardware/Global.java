package org.firstinspires.ftc.teamcode.hardware;

public class Global {
    public static  enum Hub {CONTROL_HUB, EXPANSION_HUB, BOTH}
    public static final int MOTOR_TPR = 1440;

    public static boolean USING_DASHBOARD;
    public static boolean IS_AUTO;

    public static boolean IS_SCORING = false;

    public static void startScoring() {
        IS_SCORING = true;
    }

    public static void stopScoring() {
        IS_SCORING = false;
    }
}
