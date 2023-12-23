package org.firstinspires.ftc.teamcode.Controllers;

public class Feedforward {
    private double Kf, current_output;

    public Feedforward(double Kf) {
        this.Kf = Kf;
    }

    public double update(double target) {
        current_output = target * Kf;
        return current_output;
    }

    public double getOutput() { return current_output; }

    public void set(double Kf) { this.Kf = Kf; }
}
