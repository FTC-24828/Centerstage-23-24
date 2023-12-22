package org.firstinspires.ftc.teamcode.Controllers;

public class Feedforward {
    public double Kf, currentOut;

    public Feedforward(double Kf) {
        this.Kf = Kf;
    }

    public double update(double target) {
        currentOut = target * Kf;
        return currentOut;
    }

    public double getOutput() { return currentOut; }

    public void set(double Kf) { this.Kf = Kf; }
}
