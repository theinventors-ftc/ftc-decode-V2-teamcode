package org.firstinspires.ftc.teamcode.Mechanisms;

public class VectorMath {
    public static Vector add_vectors(Vector v1, Vector v2) {
        return new Vector(v1.getVx() + v2.getVx(), v1.getVy() + v2.getVy(), false);
    }

    public static Vector add_vectors(Vector... vectors) {

        Vector sum = new Vector(0, 0);

        for (Vector vec : vectors) sum = add_vectors(sum, vec);

        return sum;
    }

    public static Vector subtract_vectors(Vector v1, Vector v2) {
        return new Vector(v1.getVx() - v2.getVx(), v1.getVy() - v2.getVy(), false);
    }

    public static Vector subtract_vectors(Vector... vectors) {

        Vector sum = new Vector(0, 0);

        for (Vector vec : vectors) sum = subtract_vectors(sum, vec);

        return sum;
    }

    public static Vector scale_vector(Vector v, double scalar) {
        return new Vector(v.getVx()*scalar, v.getVy()*scalar, false);
    }

    public static Vector rotate_vector(Vector v, double angle) {
        double new_angle = v.getAngle() + angle;
        return new Vector(v.getMagnitude(), new_angle, true);
    }

    public static double dot_product(Vector v1, Vector v2) {
        return v1.getVx()*v2.getVx() + v1.getVy()*v2.getVy();
    }
}
