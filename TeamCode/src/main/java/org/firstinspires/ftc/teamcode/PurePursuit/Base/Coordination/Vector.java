package org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination;

import androidx.annotation.Nullable;

public class Vector {

    private double x, y;

    public Vector (double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector (Vector vector) {
        this.x = vector.x;
        this.y = vector.y;
    }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
