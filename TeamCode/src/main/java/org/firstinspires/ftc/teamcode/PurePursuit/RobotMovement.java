package org.firstinspires.ftc.teamcode.PurePursuit;

import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.atTarget;
import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.calculateCircleIntersection;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Controllers.PIDFEx;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Vector;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.Localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotMap;

public class RobotMovement {

    /*--The Upper Grid--*/
    private RobotMap robotMap;
    private PinpointLocalizer localizer;
    private PIDFEx
        upperParallelPID,
        lowerParallelPID,
        upperPerpendicularPID,
        lowerPerpendicularPID,
        upperRotationalPID,
        lowerRotationalPID;

    /*-- Theta Interpolation --*/
    public enum ThetaInterpolation {
        TANGENTIAL,
        CONSTANT,
        HYBRID
    }

    private ThetaInterpolation thetaInterpolation;

    private double
        finalTargetTheta,
        realTranslationalEndDistance,
        realTranslationalStartDistance,
        realThetaStartDistance,
        realThetaEndDistance;

    /*-- Logic --*/
    private boolean
        isFinished,
        nullDetected,
        isReversed;

    private Vector currentTo_Point;

    /*-- Temp --*/
    private Pose
        currentPose = new Pose(0,0,0),
        followPoint = new Pose(0, 0, 0),
        realEnd = new Pose(0, 0, 0),
        goTo = new Pose(0,0,0);

    /*-- Constructor --*/
    public RobotMovement(RobotMap robotMap, Pose startingPose) {
        this.robotMap = robotMap;
        localizer = new PinpointLocalizer(robotMap, startingPose);
        initializeControllers();
    }

    private void initializeControllers() {
        upperParallelPID = new PIDFEx(RobotConstants.getUpperParallelPID());
        lowerParallelPID = new PIDFEx(RobotConstants.getLowerParallelPID());
        upperPerpendicularPID = new PIDFEx(RobotConstants.getUpperPerpendicularPID());
        lowerPerpendicularPID = new PIDFEx(RobotConstants.getLowerPerpendicularPID());
        upperRotationalPID = new PIDFEx(RobotConstants.getUpperRotationalPID());
        lowerRotationalPID = new PIDFEx(RobotConstants.getLowerRotationalPID());
    }

    private void updateControllerCoefficients() {
        upperParallelPID.setCofficients(RobotConstants.getUpperParallelPID());
        lowerParallelPID.setCofficients(RobotConstants.getLowerParallelPID());
        upperPerpendicularPID.setCofficients(RobotConstants.getUpperPerpendicularPID());
        lowerPerpendicularPID.setCofficients(RobotConstants.getLowerPerpendicularPID());
        upperRotationalPID.setCofficients(RobotConstants.getUpperRotationalPID());
        lowerRotationalPID.setCofficients(RobotConstants.getLowerRotationalPID());
    }

    private void updateControllerCoefficients(Pose confs) {
        upperParallelPID.setP(confs.getX());
        lowerParallelPID.setCofficients(RobotConstants.getLowerParallelPID());
        upperPerpendicularPID.setP(confs.getY());
        lowerPerpendicularPID.setCofficients(RobotConstants.getLowerPerpendicularPID());
        upperRotationalPID.setP(confs.getTheta());
        lowerRotationalPID.setCofficients(RobotConstants.getLowerRotationalPID());
    }

    public void updateLocalizer() {
        localizer.update();
        currentPose.setVec(localizer.getPose().getVec());
        currentPose.setTheta(MathFunction.angleWrap(localizer.getPose().getTheta()));
    }

    /*-- Async Pure Pursuit Logic --*/
    public void followPathUpdate(Pose[] points) {

        Pose motorsPower;
        followPoint = points[0];

        Pose start;
        Pose end;

        realEnd = points[points.length - 1];
        Pose realStart = points[0];

        updateLocalizer();
        updateControllerCoefficients();

        /*-- Absolut Translational Errors --*/
        realTranslationalStartDistance = Math.hypot(realStart.getX() - currentPose.getX(),
                                                    realStart.getY() - currentPose.getY());

        realTranslationalEndDistance = Math.hypot(realEnd.getX() - currentPose.getX(),
                                                  realEnd.getY() - currentPose.getY());

        /*-- Absolut Theta Errors --*/
        realThetaStartDistance = realStart.getTheta() - currentPose.getTheta();

        realThetaEndDistance = realEnd.getTheta() - currentPose.getTheta();

        double currentRadius = Range.scale(
            Math.hypot(Math.abs(localizer.getVelocity().getX()),
                       Math.abs(localizer.getVelocity().getY())),
            0,
            RobotConstants.getMaxVelocity(),
            RobotConstants.getMinRadiusRange(),
            RobotConstants.getMaxRadiusRange());

        if (atTarget(currentPose, realEnd)) {
            isFinished = true;
        }

        if (realTranslationalEndDistance <= currentRadius || isFinished()) {
            currentTo_Point = realEnd.getVec();

        } else {
            for (int i = points.length - 1; i > 0; --i) {
                end = points[i];
                start = points[i - 1];

                currentTo_Point = calculateCircleIntersection(
                    currentPose.getVec(),
                    currentRadius,
                    start.getVec(),
                    end.getVec()
                );

                if (currentTo_Point != null) {
                    nullDetected = false;
                    followPoint.setVec(currentTo_Point);
                    break;

                } else {
                    nullDetected = true;
                }
            }
        }

        if (currentTo_Point != null) {
            updateControllerCoefficients(
                calculateCurrentCoefficients(realTranslationalStartDistance, realTranslationalEndDistance,
                                             realThetaStartDistance, realThetaEndDistance)
            );

            double theta = calculateCurrentTheta(currentPose, currentTo_Point, realEnd,
                                                 realTranslationalEndDistance);

            followPoint.setTheta(theta);

            motorsPower = goToPoint(followPoint, currentPose, realTranslationalEndDistance, realThetaEndDistance);
            goTo = turnToRobotCentric(motorsPower, currentPose);
        }
    }

    /*-- Control Magic --*/
    public Pose goToPoint(Pose pose, Pose currentPose, double error, double thetaError) {
        Pose answers = new Pose(0,0,0);

        Pose fixedFollowPose = new Pose(pose.getVec(),
                                   MathFunction.angleErrorWrap(pose.getTheta(),
                                                               localizer.getPose().getTheta()));

        Pose fixedCurrentPose = new Pose(currentPose.getVec(),
                                        MathFunction.angleErrorWrap(currentPose.getTheta(),
                                                                    localizer.getPose().getTheta()));

        upperParallelPID.setSetPoint(fixedFollowPose.getX());
        lowerParallelPID.setSetPoint(fixedFollowPose.getX());
        upperPerpendicularPID.setSetPoint(fixedFollowPose.getY());
        lowerPerpendicularPID.setSetPoint(fixedFollowPose.getY());
        upperRotationalPID.setSetPoint(fixedFollowPose.getTheta());
        lowerRotationalPID.setSetPoint(fixedFollowPose.getTheta());

        if (error <= RobotConstants.getLowerPIDThreshold_X()) {
            answers.setX(lowerParallelPID.calculate(fixedCurrentPose.getX()));
        } else {
            answers.setX(upperParallelPID.calculate(fixedCurrentPose.getX()));
        }

        if (error <= RobotConstants.getLowerPIDThreshold_Y()) {
            answers.setY(lowerPerpendicularPID.calculate(fixedCurrentPose.getY()));
        } else {
            answers.setY(upperPerpendicularPID.calculate(fixedCurrentPose.getY()));
        }

        if (Math.abs(thetaError) <= RobotConstants.getRotationalLowerPIDThreshold()) {
            answers.setTheta(lowerRotationalPID.calculate(currentPose.getTheta()));
        } else {
            answers.setTheta(upperRotationalPID.calculate(fixedCurrentPose.getTheta()));
        }

        return answers;
    }

    /*-- Velocity Control Magic --*/
    public Pose calculateCurrentCoefficients(double errorStart, double errorEnd,
                                             double thetaErrorStart, double thetaErrorEnd) {
        Pose answer = new Pose(0,0,0);

        double distEnd_X = errorEnd - RobotConstants.getLowerPIDThreshold_X();
        double distEnd_Y = errorEnd - RobotConstants.getLowerPIDThreshold_Y();
        double distEnd_theta = thetaErrorEnd - RobotConstants.getRotationalLowerPIDThreshold();

        double trigger_X =
            (RobotConstants.getUpperParallelPID().getkP() - RobotConstants.getLowerParallelPID().getkP()) / RobotConstants.getMaxDecceleration();
        double trigger_Y =
            (RobotConstants.getUpperPerpendicularPID().getkP() - RobotConstants.getLowerPerpendicularPID().getkP()) / RobotConstants.getMaxDecceleration();
        double trigger_Theta =
            (RobotConstants.getUpperRotationalPID().getkP() - RobotConstants.getLowerRotationalPID().getkP()) / RobotConstants.getMaxRotationalDecceleration();

        if (errorEnd <= trigger_X) {
            answer.setX(Range.clip(
                RobotConstants.getMaxDecceleration() * distEnd_X + RobotConstants.getLowerParallelPID().getkP(),
                RobotConstants.getLowerParallelPID().getkP(),
                RobotConstants.getUpperParallelPID().getkP()
            ));
        } else {
            answer.setX(Range.clip(
                RobotConstants.getMaxAcceleration() * errorStart + RobotConstants.getLowerParallelPID().getkP(),
                RobotConstants.getLowerParallelPID().getkP(),
                RobotConstants.getUpperParallelPID().getkP()
            ));
        }

        if (errorEnd <= trigger_Y) {
            answer.setY(Range.clip(
                RobotConstants.getMaxDecceleration() * distEnd_Y + RobotConstants.getLowerPerpendicularPID().getkP(),
                RobotConstants.getLowerPerpendicularPID().getkP(),
                RobotConstants.getUpperPerpendicularPID().getkP()
            ));
        } else {
            answer.setY(Range.clip(
                RobotConstants.getMaxAcceleration() * errorStart + RobotConstants.getLowerPerpendicularPID().getkP(),
                RobotConstants.getLowerPerpendicularPID().getkP(),
                RobotConstants.getUpperPerpendicularPID().getkP()
            ));
        }

        if (thetaErrorEnd <= trigger_Theta) {
            answer.setTheta(Range.clip(
                RobotConstants.getMaxRotationalDecceleration() * distEnd_theta + RobotConstants.getLowerRotationalPID().getkP(),
                RobotConstants.getLowerRotationalPID().getkP(),
                RobotConstants.getUpperRotationalPID().getkP()
            ));
        } else {
            answer.setTheta(Range.clip(
                RobotConstants.getMaxRotationalAcceleration() * thetaErrorStart + RobotConstants.getLowerRotationalPID().getkP(),
                RobotConstants.getLowerRotationalPID().getkP(),
                RobotConstants.getUpperRotationalPID().getkP()
            ));
        }

        return answer;
    }

    private double calculateCurrentTheta(Pose currentPose, Vector currentTo_Point, Pose realEnd,
                                         double realTranslationalEndDistance) {
        switch (thetaInterpolation) {
            case CONSTANT:
                break;

            case TANGENTIAL:
                finalTargetTheta = Math.toDegrees(Math.atan2(
                        currentTo_Point.getY() - currentPose.getY(),
                        currentTo_Point.getX() - currentPose.getX()
                ));
                break;

            case HYBRID:
                if (realTranslationalEndDistance <= RobotConstants.getHybridThetaDistanceThreshold()) {
                    finalTargetTheta = realEnd.getTheta();
                } else {
                    finalTargetTheta = Math.toDegrees(Math.atan2(
                        currentTo_Point.getY() - currentPose.getY(),
                        currentTo_Point.getX() - currentPose.getX()
                    ));
                }
                break;
        }

        ///////
        if (isReversed) {
            finalTargetTheta = MathFunction.angleWrap(finalTargetTheta - 180);
        }

        return finalTargetTheta;
    }

    /*-- Util --*/
    public Pose turnToRobotCentric(Pose pose, Pose currentPose) {
        Pose fixedPose = new Pose(pose.getY(), pose.getX(), -pose.getTheta());

        double fixedTheta = Math.toRadians(currentPose.getTheta());

        double rotX =
            fixedPose.getX() * Math.cos(fixedTheta) - fixedPose.getY() * Math.sin(fixedTheta);
        double rotY =
            fixedPose.getX() * Math.sin(fixedTheta) + fixedPose.getY() * Math.cos(fixedTheta);

        return new Pose(-rotX, rotY, fixedPose.getTheta());
    }

    public void breakFollowing() {
        isFinished = true;
    }

    /*-- Functions --*/
    public boolean isFinished() {
        return isFinished;
    }

    public boolean isNullDetected() {
        return nullDetected;
    }

    public void setThetaInterpolation(ThetaInterpolation set) {
        if (set == ThetaInterpolation.CONSTANT) {
            throw new RuntimeException("Connot set Constant Interpolation from this function");
        } else {
            thetaInterpolation = set;
        }
    }

    public void setConstantThetaInterpolation(double set) {
        thetaInterpolation = ThetaInterpolation.CONSTANT;
        finalTargetTheta = set;
    }

    public Pose getCurrentPose() {
        return currentPose;
    }

    public void setCurrentPose(Pose set) {
        localizer.setPose(set);
        currentPose = set;
    }

    public double getFinalTargetTheta() {
        return finalTargetTheta;
    }

    public double getRealTranslationalStartDistance() {
        return realTranslationalStartDistance;
    }

    public double getRealTranslationalEndDistance() {
        return realTranslationalEndDistance;
    }

    public double getRealThetaStartDistance() {
        return realThetaStartDistance;
    }

    public double getRealThetaEndDistance() {
        return realThetaEndDistance;
    }

    public void setReversed(boolean set) {
        isReversed = set;
    }

    public Pose getFollowPoint () {
        return followPoint;
    }

    public Pose getRealEnd() {
        return realEnd;
    }

    public void reset() {
        isFinished = false;
        nullDetected = false;
        followPoint = new Pose(0, 0, 0);
        realEnd = new Pose(0, 0, 0);
        currentTo_Point = null;

        upperParallelPID.reset();
        lowerParallelPID.reset();
        upperPerpendicularPID.reset();
        lowerPerpendicularPID.reset();
        upperRotationalPID.reset();
        lowerRotationalPID.reset();
    }

    public Pose getPowers() {
        return goTo;
    }
}