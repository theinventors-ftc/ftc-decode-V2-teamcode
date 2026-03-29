package org.firstinspires.ftc.teamcode.Mechanisms;

public class Vector {
    private double magnitude, angle;

    public Vector(double v1, double v2, boolean polar) { // (R, θ) or (X, Y)
        if(polar) {
            this.magnitude = v1;
            this.angle = v2;
            return;
        }

        magnitude = Math.hypot(v1, v2);
        angle = Math.atan2(v2, v1);
    }

    public Vector(double v1, double v2) {
        this(v1, v2, true);
    }

    public double getMagnitude() {
        return magnitude;
    }

    public double getAngle() {
        return angle;
    }

    public double getVx() {
        return magnitude*Math.cos(angle);
    }

    public double getVy() {
        return magnitude*Math.sin(angle);
    }
}
