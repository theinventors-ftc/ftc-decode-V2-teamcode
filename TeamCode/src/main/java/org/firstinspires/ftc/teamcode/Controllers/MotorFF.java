package org.firstinspires.ftc.teamcode.Controllers;

public class MotorFF {
    public  double ks, kv, ka;

    public MotorFF(double ks, double kv, double ka) {
        this.ks = ks;
        this.kv = kv;
        this.ka = ka;
    }

    public MotorFF(double ks, double kv) {
        this(ks, kv, 0);
    }

    public double calculate(double velocity, double acceleration) {
        return ks * Math.signum(velocity) + kv * velocity + ka * acceleration;
    }

    public double calculate(double velocity) {
        return calculate(velocity, 0);
    }

    public double maxAchievableVelocity(double maxVoltage, double acceleration) {
        return (maxVoltage - ks - acceleration * ka) / kv;
    }

    public double minAchievableVelocity(double maxVoltage, double acceleration) {
        return (-maxVoltage + ks - acceleration * ka) / kv;
    }

    public double maxAchievableAcceleration(double maxVoltage, double velocity) {
        return (maxVoltage - ks * Math.signum(velocity) - velocity * kv) / ka;
    }

    public double minAchievableAcceleration(double maxVoltage, double velocity) {
        return maxAchievableAcceleration(-maxVoltage, velocity);
    }

    public void setKs(double ks) {
        this.ks = ks;
    }

    public void setKv(double kv) {
        this.kv = kv;
    }

    public void setKa(double ka) {
        this.ka = ka;
    }
}

