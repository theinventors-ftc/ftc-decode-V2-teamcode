package org.firstinspires.ftc.teamcode.Controllers;

import org.firstinspires.ftc.teamcode.Util.Timer;

public class SigmoidPositionProfile {
    private double duration, k, start, target, t;
    private Timer timer;

    public SigmoidPositionProfile(double duration, double k) {
        this.duration = duration;
        this.k = k;
        timer = new Timer();
    }

    public SigmoidPositionProfile(double k) {
        this.duration = 1; // fix
        this.k = k;
        timer = new Timer();
    }

    public void reset(double currentPos, double target) {
        this.start = currentPos;
        this.target = target;
        this.duration = (Math.abs(this.target - this.start)/90.0)*0.6;
        this.t = 0;
    }

    public void update() {
        t = Math.min(t + timer.getElapsedTimeSeconds(), duration);
        timer.resetTimer();
    }

    public double getPosition() {
        double x = (t / duration - 0.5) * 12;
        double s = 1.0 / (1.0 + Math.exp(-k * x));
        return start + (target - start) * s;
    }

    public void setDuration(double duration) {
        this.duration = duration;
    }

    public void setK(double k) {
        this.k = k;
    }

    public double getDuration() {
        return duration;
    }
}
