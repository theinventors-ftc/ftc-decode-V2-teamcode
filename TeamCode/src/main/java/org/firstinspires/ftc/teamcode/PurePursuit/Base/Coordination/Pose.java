package org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination;

public class Pose {

    private double x, y, theta;

    public Pose (Pose pose) {
        this.x = pose.x;
        this.y = pose.y;
        this.theta = pose.theta;
    }

    public Pose (double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public Pose (double x, double y) {
        this.x = x;
        this.y = y;
        this.theta = 0;
    }

    public Pose (Vector vector, double theta) {
        setVec(vector);
        this.theta = theta;
    }

    public void setVec(Vector vector) {
        this.x = vector.getX();
        this.y = vector.getY();
    }

    public Vector getVec() {
        return new Vector(x, y);
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setTheta(double theta) {
        this.theta = theta;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public void plusX(double x) {
        this.x += x;
    }

    public void plusY(double y) {
        this.y += y;
    }

    public void plusTheta(double theta) {
        this.theta += theta;
    }

    public void plusVec(Vector vec) {
        plusX(vec.getX());
        plusY(vec.getY());
    }

    public void plusPose(Pose pose) {
        plusVec(pose.getVec());
        plusTheta(pose.theta);
    }

    public Pose get() {
        return this;
    }
}
