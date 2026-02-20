package org.firstinspires.ftc.teamcode.PurePursuit.Base.Math;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Vector;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants;

public class MathFunction {

    public static double angleWrap(double angle) {
        double analogAngle = (angle / 360) % 1;

        if (analogAngle < 0) {
            return (analogAngle * 360) + 360;
        }

        return analogAngle * 360;
    }

    public static double wrapDegrees(double angleDeg) { // angle -> (-180, 180)
        angleDeg = angleDeg % 360.0;
        if (angleDeg > 180.0) {
            angleDeg -= 360.0;
        } else if (angleDeg <= -180.0) {
            angleDeg += 360.0;
        }
        return angleDeg;
    }

    public static double angleErrorWrap(double angle, double gyroValueDouble) {
        int minDistIdx, maxIdx;

        maxIdx = (int) Math.ceil(gyroValueDouble / 360);
        if (Math.abs((maxIdx - 1) * 360 + angle - gyroValueDouble) > Math.abs((maxIdx) * 360 + angle - gyroValueDouble))
            minDistIdx = maxIdx;
        else
            minDistIdx = maxIdx - 1;

        return minDistIdx * 360 + angle;
    }

    public static double dot(double[] a, double[] b) {
        if (a.length != b.length) {
            RobotLog.e("Vectors must be the same length");
        }
        double sum = 0;
        for (int i = 0; i < a.length; i++) {
            sum += a[i] * b[i];
        }
        return sum;
    }

    public static Vector calculateCircleIntersection(Vector state, double radius,
                                                                     Vector start, Vector end) {

        double[] d = {end.getX() - start.getX(), end.getY() - start.getY()};

        // Calculate the differences in x, y between the robot's position and the segment's start
        double[] f = {start.getX() - state.getX(), start.getY() - state.getY()};

        // Coefficients for the quadratic equation (a*t^2 + b*t + c = 0)
        double a = dot(d, d);
        double b = 2 * dot(f, d);
        double c = dot(f, f) - radius * radius;

        // Calculate the discriminant to check if the circle intersects with the segment
        double discriminant = b * b - 4 * a * c;

        // If discriminant is negative, no intersection; emergency route
        if (discriminant < 0) {
            return null;
        } else {
            // Calculate the two possible values of t where intersections occur
            discriminant = Math.sqrt(discriminant);
            double t1 = (-b + discriminant) / (2 * a);
            double t2 = (-b - discriminant) / (2 * a);
            double t = Math.max(t1, t2);

            if (t < 0.0 || t > 1.0) {
                return null;
            }

            double lx = start.getX() + t * d[0];
            double ly = start.getY() + t * d[1];

            return new Vector(lx, ly);
        }
    }

    public static boolean atTarget(Pose currentPose, Pose targetPoint) {
        return Math.abs(targetPoint.getX() - currentPose.getX()) <= RobotConstants.getX_Threshold()
            && Math.abs(targetPoint.getY() - currentPose.getY()) <= RobotConstants.getY_Threshold()
            && Math.abs(targetPoint.getTheta() - currentPose.getTheta()) <= RobotConstants.getTheta_Threshold();
    }

    public static double inToMM(double in) {
        return in * 25.4;
    }

    public static double mmToIn(double mm) {
        return mm / 25.4;
    }

    public static double normalizeAngle(double angleDegrees) {
        double angle = angleDegrees % 360;
        if (angle < 0) {
            return angle + 360;
        }
        return angle;
    }

    public static double getSmallestAngleDifference(double one, double two) {
        return Math.min(normalizeAngle(one - two), normalizeAngle(two - one));
    }

    public static Pose subtractPoses(Pose one, Pose two) {
        return new Pose(one.getX() - two.getX(), one.getY() - two.getY(),
                        one.getTheta() - two.getTheta());
    }
}