package org.firstinspires.ftc.teamcode.Controllers;

public class Feedforward {
    public double Kf, currentOut;

    public Feedforward(double Kf) {
        this.Kf = Kf;
    }

    public double update(double input) {
        currentOut = input * Kf;
        return currentOut;
    }

    public double getOutput() { return currentOut; }
}
