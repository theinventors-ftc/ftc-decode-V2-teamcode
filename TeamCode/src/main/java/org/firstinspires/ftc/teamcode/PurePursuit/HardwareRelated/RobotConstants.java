package org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Controllers.PIDFExCoeffs;

@Config
public class RobotConstants {

    private static double

        /*-- Localization --*/
        forwardY = 5.2362204724409,
        strafeX = -4.7244094488189,

        /*-- Robot Movement --*/
        maxParallelVelocity = 76, // max target velocity for the path follow (ticks per second)
        maxParallelAcceleration = 20, // max target acceleration and deceleration for the path follow
        // (ticks per second)
        maxParallelDecceleration = 20, // max target acceleration and deceleration for the path follow
        // (ticks per second)
                                // POSITIVE ONLY

        maxPerpendicularVelocity = 54, // max target velocity for the path follow (ticks per second)
        maxPerpendicularAcceleration = 20, // max target acceleration and deceleration for the path
    // follow
        // (ticks per second)
        maxPerpendicularDecceleration = 20, // max target acceleration and deceleration for the path
    // follow
        // (ticks per second)
                                // POSITIVE ONLY

        maxRotationalVelocity = 341, // max rotational target velocity for the path follow
        maxRotationalAcceleration = 15, // max rotational target acceleration and deceleration for the path follow
        maxRotationalDecceleration = 15, // max rotational target acceleration and deceleration for the path follow
                                // POSITIVE ONLY

        minRadiusRange = 1, // min lookahead distance (inches)
        maxRadiusRange = 5, // max lookahead distance (inches)

        xThreshold = 3, // threshold for X axis in the path following algorithm
        yThreshold = 3, // threshold for Y axis in the path following algorithm
        thetaThreshold = 3, // threshold for Theta(rotational) axis in the path following algorithm

        hybridThetaDistanceThreshold = 7, // absolut distance threshold of the hybrid theta interpolation

        lowerPIDThreshold_Forward = 4, // inches from ending target to activate the lower PIDS
        lowerPIDThreshold_Strafe = 4, // inches from ending target to activate the lower PIDS
        rotationalLowerPIDThreshold = 0, // degrees from ending target to activate the lower PID

        robotX = 15, // robot's size in the x axis
        robotY = 15; // robot's size in the y axis

    /*-- Follower --*/
    public static PIDFExCoeffs

        upperParallelPID = new PIDFExCoeffs(
            0.1,
            0.0,
            0.015,
            0,
            0.1,
            0.0,
            0.0,
            0.0
        ),
        lowerParallelPID = new PIDFExCoeffs(
            0.2,
            0.008,
            0.022,
            0,
            0.1,
            0.0,
            lowerPIDThreshold_Forward,
            0.6
        ),
        upperPerpendicularPID = new PIDFExCoeffs(
            0.1,
            0.0,
            0.01,
            0,
            0.0,
            0.0,
            0.0,
            0.0
        ),
        lowerPerpendicularPID = new PIDFExCoeffs(
            0.05,
            0.12,
            0.02,
            0,
            0.0,
            0.0,
            lowerPIDThreshold_Strafe,
            0.6
        ),
        upperRotationalPID = new PIDFExCoeffs(
            0.05,
            0.0,
            0.0025,
            0,
            0.1,
            0.0,
            0.0,
            0.0
        ),
        lowerRotationalPID = new PIDFExCoeffs(
            0.032,
            0.09,
            0.0015,
            0,
            0.0,
            0.0,
            rotationalLowerPIDThreshold,
            0.6
        );

    /*-- Meow --*/
    public static void setMaxParallelVelocity (double set) {
        RobotConstants.maxParallelVelocity = set;
    }
    public static void setMaxParallelAcceleration (double set) {
        RobotConstants.maxParallelAcceleration = set;
    }
    public static void setMaxParallelDecceleration (double set) {
        RobotConstants.maxParallelDecceleration = set;
    }
    public static void setMaxPerpendicularVelocity (double set) {
        RobotConstants.maxPerpendicularVelocity = set;
    }
    public static void setMaxPerpendicularAcceleration (double set) {
        RobotConstants.maxPerpendicularAcceleration = set;
    }
    public static void setMaxPerpendicularDecceleration (double set) {
        RobotConstants.maxPerpendicularDecceleration = set;
    }
    public static void setMaxRotationalVelocity(double set) {
        RobotConstants.maxRotationalVelocity = set;
    }
    public static void setMaxRotationalAcceleration(double set) {
        RobotConstants.maxRotationalAcceleration = set;
    }
    public static void setMaxRotationalDecceleration(double set) {
        RobotConstants.maxRotationalDecceleration = set;
    }
    public static void setMinRadiusRange(double set) {
        RobotConstants.minRadiusRange = set;
    }
    public static void setMaxRadiusRange(double set) {
        RobotConstants.maxRadiusRange = set;
    }
    public static void setRobotX(double set) {
        RobotConstants.robotX = set;
    }
    public static void setRobotY(double set) {
        RobotConstants.robotY = set;
    }
    public static void setForwardY(double set) {
        RobotConstants.forwardY = set;
    }
    public static void setStrafeX(double set) {
        RobotConstants.strafeX = set;
    }
    public static void setX_Threshold (double set) {
        RobotConstants.xThreshold = set;
    }
    public static void setY_Threshold (double set) {
        RobotConstants.yThreshold = set;
    }
    public static void setTheta_Threshold (double set) {
        RobotConstants.thetaThreshold = set;
    }
    public static void setHybridThetaDistanceThreshold(double set) {
        RobotConstants.hybridThetaDistanceThreshold = set;
    }
    public static void setUpperParallelPID(PIDFExCoeffs set) {
        RobotConstants.upperParallelPID = set;
    }
    public static void setLowerParallelPID(PIDFExCoeffs set) {
        RobotConstants.lowerParallelPID = set;
    }
    public static void setUpperPerpendicularPID(PIDFExCoeffs set) {
        RobotConstants.upperPerpendicularPID = set;
    }
    public static void setLowerPerpendicularPID(PIDFExCoeffs set) {
        RobotConstants.lowerPerpendicularPID = set;
    }
    public static void setUpperRotationalPID(PIDFExCoeffs set) {
        RobotConstants.upperRotationalPID = set;
    }
    public static void setLowerRotationalPID(PIDFExCoeffs set) {
        RobotConstants.lowerRotationalPID = set;
    }
    public static void setLowerPIDThreshold_Forward (double set) {
        RobotConstants.lowerPIDThreshold_Forward = set;
    }
    public static void setLowerPIDThreshold_Strafe (double set) {
        RobotConstants.lowerPIDThreshold_Strafe = set;
    }
    public static void setRotationalLowerPIDThreshold(double set) {
        RobotConstants.rotationalLowerPIDThreshold = set;
    }

    /*-- Meow meow --*/
    public static double getMaxParallelVelocity () {
        return maxParallelVelocity;
    }
    public static double getMaxParallelAcceleration () {
        return maxParallelAcceleration;
    }
    public static double getMaxParallelDecceleration () {
        return maxParallelDecceleration;
    }
    public static double getMaxPerpendicularVelocity () {
        return maxPerpendicularVelocity;
    }
    public static double getMaxPerpendicularAcceleration () {
        return maxPerpendicularAcceleration;
    }
    public static double getMaxPerpendicularDecceleration () {
        return maxPerpendicularDecceleration;
    }
    public static double getMaxRotationalVelocity() {
        return maxRotationalVelocity;
    }
    public static double getMaxRotationalAcceleration() {
        return maxRotationalAcceleration;
    }
    public static double getMaxRotationalDecceleration() {
        return maxRotationalDecceleration;
    }
    public static double getMinRadiusRange() {
        return minRadiusRange;
    }
    public static double getMaxRadiusRange() {
        return maxRadiusRange;
    }
    public static double getRobotX() {
        return robotX;
    }
    public static double getRobotY() {
        return robotY;
    }
    public static double getForwardY() {
        return forwardY;
    }
    public static double getStrafeX() {
        return strafeX;
    }
    public static double getX_Threshold () {
        return xThreshold;
    }
    public static double getY_Threshold () {
        return yThreshold;
    }
    public static double getTheta_Threshold () {
        return thetaThreshold;
    }
    public static double getHybridThetaDistanceThreshold() {
        return hybridThetaDistanceThreshold;
    }
    public static PIDFExCoeffs getUpperParallelPID() {
        return upperParallelPID;
    }
    public static PIDFExCoeffs getLowerParallelPID() {
        return lowerParallelPID;
    }
    public static PIDFExCoeffs getUpperPerpendicularPID() {
        return upperPerpendicularPID;
    }
    public static PIDFExCoeffs getLowerPerpendicularPID() {
        return lowerPerpendicularPID;
    }
    public static PIDFExCoeffs getUpperRotationalPID() {
        return upperRotationalPID;
    }
    public static PIDFExCoeffs getLowerRotationalPID() {
        return lowerRotationalPID;
    }
    public static double getLowerPIDThreshold_Forward () {
        return lowerPIDThreshold_Forward;
    }
    public static double getLowerPIDThreshold_Strafe () {
        return lowerPIDThreshold_Strafe;
    }
    public static double getRotationalLowerPIDThreshold() {
        return rotationalLowerPIDThreshold;
    }
}