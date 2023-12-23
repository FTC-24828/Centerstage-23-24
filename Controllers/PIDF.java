package org.firstinspires.ftc.teamcode.Controllers;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.teamcode.Dashboard.PIDConstants;

public class PIDF {
    static private ElapsedTime timer = new ElapsedTime();
    TelemetryMessage telemetry = new TelemetryMessage();
//    TelemetryMessage telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    public double Kp, Ki, Kd, Kf, intLim, maxErr;

    public double prev_estimate = 0, filter = 0, lastError = 0, lastTarget, integral = 0;

    public double current_output;       //for debugging output

    // gain values for proportional, integral, derivative, and noise filter
    // lim is the integral cap and maxErr determines acceptable error range
    public PIDF (double Kp, double Ki, double Kd, double Kf, double lim, double maxErr) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
        this.intLim = lim;
        this.maxErr = maxErr;
    }

    public void set(double Kp, double Ki, double Kd, double Kf, double lim, double maxErr) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.Kf = Kf;
        this.intLim = lim;
        this.maxErr = maxErr;
    }

    // return and output based on the current state vs the target state
    public double update(double current, double target) {
        if (target != lastTarget) this.reset(target);
        double error = target - current;
        if (error <= maxErr && error >= -maxErr) return 0;  //acceptable error range exit

        //integral calculation with integral limit
        integral += (error * timer.seconds());
        if (integral > intLim) integral = intLim;
        if (integral < -intLim) integral = -intLim;

        //noise filter for derivative
        filter = Kf * prev_estimate + (1 - Kf) * (error - lastError);
        prev_estimate = filter;

        double output = Kp * error + Ki * integral + Kd * filter / timer.seconds();

        //set error for next iteration
        lastError = error;
        current_output = output; //debugging purposes
        timer.reset();
        return output;
    }

    public void reset(double target) {
        integral = 0; lastError = 0; lastTarget = target;
        prev_estimate = 0; filter = 0;
    }

    public void printAll() {
        telemetry.addData("Last error", lastError);
        telemetry.addData("Last target", lastTarget);
        telemetry.addData("integral", integral);
        telemetry.addData("current output", current_output);
    }

    public static PIDF create(double Kp, double Ki, double Kd, double Kf, double lim, double maxErr) {
        PIDF obj = new PIDF(Kp, Ki, Kd, Kf, lim, maxErr);
        return obj;
    }

    public static PIDF create(double Kp, double Ki, double Kd) {
        PIDF obj = new PIDF(Kp, Ki, Kd, 100, 0, 0);
        return obj;
    }

    public static PIDF create(double Kp, double Ki, double Kd, double Kf, double lim) {
        PIDF obj = new PIDF(Kp, Ki, Kd, Kf, lim, 0);
        return obj;
    }

    public void set(PIDConstants obj) {
        this.Kp = obj.Kp;
        this.Kd = obj.Kd;
        this.Ki = obj.Ki;
        this.Kf = obj.Kf;
        this.intLim = obj.lim;
        this.maxErr = obj.maxErr;
    }
}