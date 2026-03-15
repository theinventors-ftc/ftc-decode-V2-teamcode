package org.firstinspires.ftc.teamcode.Controllers;

public class PIDFExCoeffs {

    private double
        kP,
        kI,
        kD,
        kF,
        alpha,
        deadzone,
        integralWorkingBounds = Double.POSITIVE_INFINITY,
        integralClippingBounds = Double.POSITIVE_INFINITY;

    public PIDFExCoeffs(double kP,
                        double kI,
                        double kD,
                        double kF,
                        double alpha,
                        double deadzone,
                        double integralWorkingBounds,
                        double integralClippingBounds)
    {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.alpha = alpha;
        this.deadzone = deadzone;
        this.integralWorkingBounds = integralWorkingBounds;
        this.integralClippingBounds = integralClippingBounds;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public void setkΙ(double kΙ) {
        this.kI = kΙ;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

    public void setAlpha(double alpha) {
        this.alpha = alpha;
    }

    public void setDeadzone(double deadzone) {
        this.deadzone = deadzone;
    }

    public void setIntegralWorkingBounds(double integralWorkingBounds) {
        this.integralWorkingBounds = integralWorkingBounds;
    }

    public void setIntegralClippingBounds(double integralClippingBounds) {
        this.integralClippingBounds = integralClippingBounds;
    }

    public double getkP() {
        return kP;
    }

    public double getkI() {
        return kI;
    }

    public double getkD() {
        return kD;
    }

    public double getkF() {
         return kF;
    }

    public double getAlpha() {
        return alpha;
    }

    public double getDeadzone() {
        return deadzone;
    }

    public double getIntegralWorkingBounds() {
        return integralWorkingBounds;
    }

    public double getIntegralClippingBounds() {
        return integralClippingBounds;
    }

    public PIDFExCoeffs get() {
        return this;
    }
}
