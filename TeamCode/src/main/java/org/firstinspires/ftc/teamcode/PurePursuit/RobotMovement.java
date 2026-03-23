package org.firstinspires.ftc.teamcode.PurePursuit;

import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.atTarget;
import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.calculateCircleIntersection;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.PIDFEx;
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

    /*-- Pure Pursuit Type --*/
    public enum Type {
        DEFAULT,
        ENGAGED
    }

private Type type = Type.ENGAGED;

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
        realThetaEndDistance,
        realPerpendicularEndDistance,
        realParallelEndDistance,
        realPerpendicularStartDistance,
        realParallelStartDistance;

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
        goTo = new Pose(0,0,0),
        currentVelocity = new Pose(0,0,0);

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

    public void updateControllerCoefficients() {
        upperParallelPID.setCofficients(RobotConstants.getUpperParallelPID());
        lowerParallelPID.setCofficients(RobotConstants.getLowerParallelPID());
        upperPerpendicularPID.setCofficients(RobotConstants.getUpperPerpendicularPID());
        lowerPerpendicularPID.setCofficients(RobotConstants.getLowerPerpendicularPID());
        upperRotationalPID.setCofficients(RobotConstants.getUpperRotationalPID());
        lowerRotationalPID.setCofficients(RobotConstants.getLowerRotationalPID());
    }

    public void updateLocalizer() {
        localizer.update();
        currentPose = localizer.getPose();
//        currentPose.setTheta(MathFunction.angleWrap(localizer.getPose().getTheta()));
        currentVelocity = localizer.getVelocity();
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

        realPerpendicularEndDistance = realEnd.getY() - currentPose.getY();
        realParallelEndDistance = realEnd.getX() - currentPose.getX();
        realPerpendicularStartDistance = realStart.getY() - currentPose.getY();
        realParallelStartDistance = realStart.getX() - currentPose.getX();

        /*-- Absolut Theta Errors --*/
        realThetaStartDistance = realStart.getTheta() - currentPose.getTheta();

        realThetaEndDistance = realEnd.getTheta() - currentPose.getTheta();

        double currentRadius = Range.scale(
            Math.hypot(Math.abs(localizer.getVelocity().getX()),
                       Math.abs(localizer.getVelocity().getY())),
            0,
            RobotConstants.getMaxParallelVelocity(),
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

            double theta = MathFunction.oneEightyToThreesixty(
                calculateCurrentTheta(
                    currentPose, currentTo_Point, realEnd, realTranslationalEndDistance
                ));

            followPoint = turnToRobotCentric(followPoint, currentPose);

            followPoint.setTheta(theta);

            motorsPower = goToPoint(followPoint, currentPose, realPerpendicularEndDistance,
                                    realParallelEndDistance, realThetaEndDistance);

            Pose temp = motionProfile(realParallelStartDistance,
                              realPerpendicularStartDistance,
                              realParallelEndDistance,
                              realPerpendicularEndDistance,
                              realThetaStartDistance,
                              realThetaEndDistance,
                              motorsPower.getX(),
                              motorsPower.getY(),
                              motorsPower.getTheta()
            );

            if (type == Type.DEFAULT) {
                goTo = new Pose(temp.getX(), 0, temp.getTheta());
            } else {
                goTo = temp;
            }
        }
    }

    /*-- Control Magic --*/
    public Pose goToPoint(Pose targetPoint, Pose currentPose, double error_perp, double error_par,
                          double thetaError) {
        Pose answers = new Pose(0,0,0);

        upperParallelPID.setSetPoint(targetPoint.getX());
        lowerParallelPID.setSetPoint(targetPoint.getX());
        upperPerpendicularPID.setSetPoint(targetPoint.getY());
        lowerPerpendicularPID.setSetPoint(targetPoint.getY());
        upperRotationalPID.setSetPoint(targetPoint.getTheta());
        lowerRotationalPID.setSetPoint(targetPoint.getTheta());

        if (Math.abs(error_par) <= RobotConstants.getLowerPIDThreshold_Forward()) {
            answers.setX(lowerParallelPID.calculate(currentPose.getX()));
        } else {
            answers.setX(upperParallelPID.calculate(currentPose.getX()));
        }

        if (Math.abs(error_perp) <= RobotConstants.getLowerPIDThreshold_Strafe()) {
            answers.setY(lowerPerpendicularPID.calculate(currentPose.getY()));
        } else {
            answers.setY(upperPerpendicularPID.calculate(currentPose.getY()));
        }

        if (Math.abs(thetaError) <= RobotConstants.getRotationalLowerPIDThreshold()) {
            answers.setTheta(lowerRotationalPID.calculate(currentPose.getTheta(), thetaError));
        } else {
            answers.setTheta(upperRotationalPID.calculate(currentPose.getTheta(), thetaError));
        }

        return answers;
    }

    /*-- Velocity Control Magic --*/
    public Pose motionProfile (double errorStart_Par, double errorStart_Perp,
                               double errorEnd_Par, double errorEnd_Perp,
                               double errorStart_Rot, double errorEnd_Rot,
                               double pidOut_Par, double pidOut_Perp, double pidOut_Rot) {
        Pose answer = new Pose(0,0,0);

        double dir_Par = (pidOut_Par < 0) ? -1 : 1;
        double dir_Perp = (pidOut_Perp < 0) ? -1 : 1;
        double dir_Rot = (pidOut_Rot < 0) ? -1 : 1;

        double trigger_Parallel =
            (RobotConstants.getMaxParallelDecceleration() * (errorEnd_Par + errorStart_Par)) /
                (RobotConstants.getMaxParallelAcceleration() + RobotConstants.getMaxParallelDecceleration()),
            trigger_Perpendicular =
                (RobotConstants.getMaxPerpendicularDecceleration() * (errorEnd_Perp + errorStart_Perp)) /
                    (RobotConstants.getMaxPerpendicularAcceleration() + RobotConstants.getMaxPerpendicularDecceleration()),
            trigger_Theta =
                (RobotConstants.getMaxRotationalDecceleration() * (errorEnd_Rot + errorStart_Rot)) /
                    (RobotConstants.getMaxRotationalAcceleration() + RobotConstants.getMaxRotationalDecceleration());

        if (errorStart_Par >= trigger_Parallel) {
            answer.setX(Range.clip(
                RobotConstants.getMaxParallelDecceleration() * errorEnd_Par * dir_Par,
                -RobotConstants.getMaxParallelVelocity(), RobotConstants.getMaxParallelVelocity()
            ));
        } else {
            answer.setX(Range.clip(
                RobotConstants.getMaxParallelAcceleration() * errorStart_Par * dir_Par,
                -RobotConstants.getMaxParallelVelocity(), RobotConstants.getMaxParallelVelocity()
            ));
        }

        if (errorStart_Perp >= trigger_Perpendicular) {
            answer.setY(Range.clip(
                RobotConstants.getMaxPerpendicularDecceleration() * errorEnd_Perp * dir_Perp,
                -RobotConstants.getMaxPerpendicularVelocity(),
                RobotConstants.getMaxPerpendicularVelocity()
            ));
        } else {
            answer.setY(Range.clip(
                RobotConstants.getMaxPerpendicularAcceleration() * errorStart_Perp * dir_Perp,
                -RobotConstants.getMaxPerpendicularVelocity(),
                RobotConstants.getMaxPerpendicularVelocity()
            ));
        }

        if (errorStart_Rot >= trigger_Theta) {
            answer.setTheta(Range.clip(
                RobotConstants.getMaxRotationalDecceleration() * errorEnd_Rot * dir_Rot,
                -RobotConstants.getMaxRotationalVelocity(), RobotConstants.getMaxRotationalVelocity()
            ));
        } else {
            answer.setTheta(Range.clip(
                RobotConstants.getMaxRotationalAcceleration() * errorStart_Rot * dir_Rot,
                -RobotConstants.getMaxRotationalVelocity(), RobotConstants.getMaxRotationalVelocity()
            ));
        }

        answer = new Pose(
            Range.scale(answer.getX(),
                        -RobotConstants.getMaxParallelVelocity(),
                        RobotConstants.getMaxParallelVelocity(),
                        -1,
                        1
            ),
            Range.scale(answer.getY(),
                        -RobotConstants.getMaxPerpendicularVelocity(),
                        RobotConstants.getMaxPerpendicularVelocity(),
                        -1,
                        1
            ),
            Range.scale(answer.getTheta(),
                        -RobotConstants.getMaxRotationalVelocity(),
                        RobotConstants.getMaxRotationalVelocity(),
                        -1,
                        1
            )
        );

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
                    type = Type.ENGAGED;
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

    public static double norm(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /*-- Util --*/
    public Pose turnToRobotCentric(Pose pose, Pose curPose, Telemetry tele) {
                Pose fixedPose = new Pose(
                    pose.getX(),
                    pose.getY(),
                    Math.toRadians(pose.getTheta())
                ); // inchs, inchs, rads

                double fixedTheta = curPose.getTheta();

                double rotX =
                    (fixedPose.getX() - curPose.getX()) * Math.cos(fixedTheta)
                        + (fixedPose.getY() - curPose.getY()) * Math.sin(fixedTheta);
                double rotY =
                    -(fixedPose.getX() - curPose.getX()) * Math.sin(fixedTheta)
                        + (fixedPose.getY() - curPose.getY()) * Math.cos(fixedTheta);

                tele.addData("Rot X: ", rotX);
                tele.addData("Rot Y: ", rotY);
                tele.addData("fixed theta: ", Math.toDegrees(fixedTheta));

                return new Pose(rotX, rotY, pose.getTheta());
    }

    public Pose turnToRobotCentric(Pose pose, Pose curPose) {
        return new Pose(0, 0, pose.getTheta());
    }

//    public Pose turnToRobotCentric(Pose pose, Pose currentPose, Telemetry tele) {
//        // Field-centric error (target relative to robot)
//        double dx = pose.getX() - currentPose.getX();
//        double dy = pose.getY() - currentPose.getY();
//
//        // Robot heading (radians)
//        double theta = currentPose.getTheta();
//
//        // Rotate field vector into robot frame
//        double rotX =  dx * Math.cos(theta) + dy * Math.sin(theta);
//        double rotY = -dx * Math.sin(theta) + dy * Math.cos(theta);
//
//        // Heading error (wrapped)
//        double rotTheta = norm(pose.getTheta() - currentPose.getTheta());
//
//        return new Pose(rotX, rotY, Math.toDegrees(rotTheta));
//    }


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

    public void setType(Type set) {
        type = set;
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

    public Pose getCurrentVelocity() {
        return currentVelocity;
    }
}