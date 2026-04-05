package org.firstinspires.ftc.teamcode.PurePursuit;

import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.atTarget;
import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.calculateCircleIntersection;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.PIDFEx;
import org.firstinspires.ftc.teamcode.Controllers.RamseteController;
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
    private PIDFEx rotationalPID;
    private RamseteController ramsete;

    /*-- Pure Pursuit Type --*/
    public enum Type {
        DEFAULT,
        ENGAGED
    }

    private Type type = Type.ENGAGED;

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
        goTo = new Pose(0,0,0),
        currentVelocity = new Pose(0,0,0);

    /*-- Constructor --*/
    public RobotMovement(RobotMap robotMap, Pose startingPose) {
        this.robotMap = robotMap;
        localizer = new PinpointLocalizer(robotMap, startingPose);
        initializeControllers();
    }

    private void initializeControllers() {
        rotationalPID = new PIDFEx(RobotConstants.getUpperRotationalPID());
        ramsete = new RamseteController(2.0, 0.8);
    }

    public void updateControllerCoefficients() {
        rotationalPID.setCofficients(RobotConstants.getLowerRotationalPID());
    }

    public void updateLocalizer() {
        localizer.update();
        currentPose = localizer.getPose();
//        currentPose.setTheta(MathFunction.angleWrap(localizer.getPose().getTheta()));
        currentVelocity = localizer.getVelocity();
    }

    public double getThetaError(double goal, double currentTheta) {

        double deltaRad = Math.toRadians(goal - currentTheta);
        return Math.toDegrees(Math.atan2(Math.sin(deltaRad), Math.cos(deltaRad)));
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

            double theta =
                MathFunction.oneEightyToThreesixty(
                    calculateCurrentTheta(
                        currentPose,
                        currentTo_Point
                    )
                );

            followPoint.setTheta(theta);

            motorsPower = new Pose (motionProfile(
                realTranslationalStartDistance,
                realTranslationalEndDistance,
                1
                ),
                0,
                rotateToPoint(followPoint, currentPose,
                              getThetaError(followPoint.getTheta(),
                                            currentPose.getTheta()
                              )
                )
            );

            if (type == Type.DEFAULT) {
                goTo = motorsPower;
            } else {
                goTo = ramsete.calculate(
                    currentPose,
                    followPoint,
                    RobotConstants.getMaxParallelVelocity() * 0.0254,
                    RobotConstants.getMaxRotationalVelocity()
                                         );
            }
        }
    }

    /*-- Control Magic --*/
    public double rotateToPoint(Pose targetPoint, Pose currentPose, double thetaError) {
        rotationalPID.setSetPoint(targetPoint.getTheta());

        return rotationalPID.calculate(currentPose.getTheta(), thetaError);
    }

    /*-- Velocity Control Magic --*/
    public double motionProfile (double errorStart, double errorEnd, double dir_Par) {
        double answer;

        double trigger =
            (RobotConstants.getMaxParallelDecceleration() * (errorEnd + errorStart)) /
                (RobotConstants.getMaxParallelAcceleration() + RobotConstants.getMaxParallelDecceleration());

        if (errorStart >= trigger) {
            answer = Range.clip(
                RobotConstants.getMaxParallelDecceleration() * errorEnd * dir_Par,
                -RobotConstants.getMaxParallelVelocity(), RobotConstants.getMaxParallelVelocity()
            );
        } else {
            answer = Range.clip(
                RobotConstants.getMaxParallelAcceleration() * errorStart * dir_Par,
                -RobotConstants.getMaxParallelVelocity(), RobotConstants.getMaxParallelVelocity()
            );
        }

        return Range.scale(answer,
                           -RobotConstants.getMaxParallelVelocity(),
                           RobotConstants.getMaxParallelVelocity(),
                           -1,
                           1
        );
    }

    private double calculateCurrentTheta(Pose currentPose, Vector currentTo_Point) {

        finalTargetTheta = Math.toDegrees(Math.atan2(
                currentTo_Point.getY() - currentPose.getY(),
                currentTo_Point.getX() - currentPose.getX()
        ));

        if (isReversed) {
            finalTargetTheta = MathFunction.angleWrap(finalTargetTheta - 180);
        }

        return finalTargetTheta;
    }

//    public static double norm(double angle) {
//        while (angle > Math.PI)  angle -= 2 * Math.PI;
//        while (angle < -Math.PI) angle += 2 * Math.PI;
//        return angle;
//    }

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

                return new Pose(rotX, rotY, Math.toDegrees(pose.getTheta()));
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

    public void setConstantThetaInterpolation(double set) {
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