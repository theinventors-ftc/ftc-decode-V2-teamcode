package org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.Localization;

import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.getSmallestAngleDifference;
import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.inToMM;
import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.subtractPoses;

import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Vector;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotMap;

/**
 * This is the Pinpoint class. This class extends the Localizer superclass and is a
 * localizer that uses the two wheel odometry set up with the IMU to have more accurate heading
 * readings. The diagram below, which is modified from Road Runner, shows a typical set up.
 *
 * The view is from the top of the robot looking downwards.
 *
 * left on robot is the y positive direction
 *
 * forward on robot is the x positive direction
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||           |
 *    | ||           |  ----> left (y positive)
 *    |              |
 *    |              |
 *    \--------------/
 *           |
 *           |
 *           V
 *    forward (x positive)
 * With the pinpoint your readings will be used in mm
 * to use inches ensure to divide your mm value by 25.4
 * @author Logan Nash
 * @author Havish Sripada 12808 - RevAmped Robotics
 * @author Ethan Doak - Gobilda
 * @version 1.0, 10/2/2024
 */
public class PinpointLocalizer {
    private GoBildaPinpointDriver odo;
    private double previousHeading;
    private double totalHeading;
    private long deltaTimeNano;
    private NanoTimer timer;
    private Pose currentVelocity;
    private Pose pinpointPose;
    private boolean pinpointCooked = false;

    public PinpointLocalizer (RobotMap robotMap, Pose startingPose) {
        odo = robotMap.getOdometry();

        setOffsets(RobotConstants.getForwardY(), RobotConstants.getStrafeX());

        odo.setEncoderResolution(robotMap.getEncoderRes());
        odo.setEncoderDirections(robotMap.getForwardEncoderDirection(), robotMap.getStrafeEncoderDirection());

        resetPinpoint();

        totalHeading = 0;
        timer = new NanoTimer();
        odo.setPosition(
                new Pose(startingPose.getX(),
                        startingPose.getY(),
                        Math.toRadians(startingPose.getTheta())
                )
        );
        pinpointPose = startingPose;
        currentVelocity = new Pose(0,0,0);
        deltaTimeNano = 1;
        previousHeading = startingPose.getTheta();
    }

    public Pose getPose() {
        return pinpointPose;
    }

    public Pose getVelocity() {
        return currentVelocity.get();
    }

    public Vector getVelocityVector() {
        return currentVelocity.getVec();
    }

    public void setPose(Pose setPose) {
        odo.setPosition(new Pose(setPose.getX(), setPose.getX(), Math.toRadians(setPose.getTheta())));
        pinpointPose = setPose;
        previousHeading = setPose.getTheta();
    }

    public void setVector(Vector setVector) {
        odo.setPosition(new Pose(setVector.getX(), setVector.getY(), Math.toRadians(pinpointPose.getTheta())));
        pinpointPose = new Pose(setVector.getX(), setVector.getY(), Math.toRadians(pinpointPose.getTheta()));
        previousHeading = Math.toRadians(pinpointPose.getTheta());
    }

    public void update() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();
        odo.update();
        Pose currentPinpointPose = getPoseEstimate(odo.getPosition(), pinpointPose, deltaTimeNano);
        totalHeading += getSmallestAngleDifference(currentPinpointPose.getTheta(),
                                                                 previousHeading);
        previousHeading = currentPinpointPose.getTheta();
        Pose deltaPose = subtractPoses(currentPinpointPose, pinpointPose);
        currentVelocity = new Pose(deltaPose.getX() / (deltaTimeNano / Math.pow(10.0, 9)),
                                   deltaPose.getY() / (deltaTimeNano / Math.pow(10.0, 9)),
                                   deltaPose.getTheta() / (deltaTimeNano / Math.pow(10.0, 9)));
        pinpointPose = currentPinpointPose;
    }

    public double getTotalHeading() {
        return totalHeading;
    }

    public double getForwardMultiplier() {
        return odo.getEncoderY();
    }

    public double getLateralMultiplier() {
        return odo.getEncoderX();
    }

    public double getTurningMultiplier() {
        return odo.getYawScalar();
    }

    private void setOffsets(double xOffset, double yOffset) {
        odo.setOffsets(inToMM(xOffset), inToMM(yOffset));
    }

    public void resetIMU() {
        odo.recalibrateIMU();
    }

    private void resetPinpoint() {
        odo.resetPosAndIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private Pose getPoseEstimate(Pose pinpointEstimate, Pose currentPose, long deltaTime) {
        double x;
        double y;
        double heading;

        pinpointCooked = false;

        pinpointEstimate.setTheta(Math.toDegrees(pinpointEstimate.getTheta()));

        if (!Double.isNaN(pinpointEstimate.getX())) {
            x = pinpointEstimate.getX();
        } else {
            x = currentPose.getX() + currentVelocity.getX() * deltaTime / Math.pow(10, 9);
            pinpointCooked = true;
        }

        if (!Double.isNaN(pinpointEstimate.getY())) {
            y = pinpointEstimate.getY();
        } else {
            y = currentPose.getY() + currentVelocity.getY() * deltaTime / Math.pow(10, 9);
            pinpointCooked = true;
        }

        if (!Double.isNaN(pinpointEstimate.getTheta())) {
            heading = pinpointEstimate.getTheta();
        } else {
            heading = currentPose.getTheta() + currentVelocity.getTheta() * deltaTime / Math.pow(10, 9);
            pinpointCooked = true;
        }

        return new Pose(x, y, heading); // (95.0/95.5), (95.0/95.4)
    }

    public boolean isNAN() {
        return pinpointCooked;
    }
}
