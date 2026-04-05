package org.firstinspires.ftc.teamcode.PurePursuit.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Controllers.PIDFExCoeffs;
import org.firstinspires.ftc.teamcode.DecodeRobotV2;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.RobotMap;

//@Disabled
@Config
@Autonomous(name = "PID Test", group = "Test")
public class PIDTest extends CommandOpMode {

    private DriveConstants driveConstants;
    private DecodeRobotV2 robot;

    private RobotMap robotMap;
    private RobotMovement rm;

    public static double goal_x = 0, goal_y = 0, goal_theta = 0;
    public static double thresh_x = 0.0, thresh_y = 0.0, thresh_theta = 0.0;
    public static double
        rotational_alpha = 0.0,
        rotational_kPu = 0.05,
        rotational_kIu = 0.0,
        rotational_kDu = 0.0,
        rotational_kPl = 0.0,
        rotational_kIl = 0.0,
        rotational_kDl = 0.0;
    public static double
        parallel_alpha = 0.0,
        parallel_kPu = 0.05,
        parallel_kIu = 0.0,
        parallel_kDu = 0.0,
        parallel_kPl = 0.0,
        parallel_kIl = 0.0,
        parallel_kDl = 0.0;
    public static double
        perpendicular_alpha = 0.0,
        perpendicular_kPu = 0.05,
        perpendicular_kIu = 0.0,
        perpendicular_kDu = 0.0,
        perpendicular_kPl = 0.0,
        perpendicular_kIl = 0.0,
        perpendicular_kDl = 0.0;

    private Pose startingPose = new Pose(0,0,0);
    private Pose goal = new Pose(goal_x,goal_y,goal_theta);

    @Override
    public void initialize () {
        CommandScheduler.getInstance().reset(); // Ultra SOS
        robotMap = new RobotMap(hardwareMap, telemetry);
        rm = new RobotMovement(robotMap, startingPose);

        /*-- Drive Constants --*/
        driveConstants = new DriveConstants();

        driveConstants.frontLeftInverted = false;
        driveConstants.frontRightInverted = true;
        driveConstants.rearRightInverted = true;
        driveConstants.rearLeftInverted = false;

        driveConstants.DEFAULT_SPEED_PERC = 1.0;
        driveConstants.SLOW_SPEED_PERC = 0.7;

        robot = new DecodeRobotV2(
            robotMap, driveConstants,
            DecodeRobotV2.Alliance.RED
        );

        robot.setAutoEnabled(true);
    }

    public double getThetaError(double goal, double currentTheta) {

        double deltaRad = Math.toRadians(goal - currentTheta);
        return Math.toDegrees(Math.atan2(Math.sin(deltaRad), Math.cos(deltaRad)));
    }

    @Override
    public void run () {
        goal = new Pose(goal_x, goal_y, goal_theta);

        PIDFExCoeffs

            upperParallelPID = new PIDFExCoeffs(
            parallel_kPu,
            parallel_kIu,
            parallel_kDu,
            0,
            parallel_alpha,
            0.0,
            0.0,
            0.0
        ),
            lowerParallelPID = new PIDFExCoeffs(
                parallel_kPl,
                parallel_kIl,
                parallel_kDl,
                0,
                parallel_alpha,
                0.0,
                thresh_x,
                0.6
            ),
            upperPerpendicularPID = new PIDFExCoeffs(
                perpendicular_kPu,
                perpendicular_kIu,
                perpendicular_kDu,
                0,
                perpendicular_alpha,
                0.0,
                0.0,
                0.0
            ),
            lowerPerpendicularPID = new PIDFExCoeffs(
                perpendicular_kPl,
                perpendicular_kIl,
                perpendicular_kDl,
                0,
                perpendicular_alpha,
                0.0,
                thresh_y,
                0.6
            ),
            upperRotationalPID = new PIDFExCoeffs(
                rotational_kPu,
                rotational_kIu,
                rotational_kDu,
                0,
                rotational_alpha,
                0.0,
                0.0,
                0.0
            ),
            lowerRotationalPID = new PIDFExCoeffs(
                rotational_kPl,
                rotational_kIl,
                rotational_kDl,
                0,
                rotational_alpha,
                0.0,
                thresh_theta,
                0.6
            );

        RobotConstants.setLowerPIDThreshold_Forward(thresh_x);
        RobotConstants.setLowerPIDThreshold_Strafe(thresh_y);
        RobotConstants.setRotationalLowerPIDThreshold(thresh_theta);

        super.run();
        for (LynxModule hub : robotMap.getHubs()) hub.clearBulkCache();

        rm.updateLocalizer();
        Pose currentPose = rm.getCurrentPose();

//        double realTranslationalEndDistance = Math.hypot(goal.getX() - currentPose.getX(),
//                                                         goal.getY() - currentPose.getY());



        double realThetaEndDistance = getThetaError(goal.getTheta(), currentPose.getTheta());

        Pose robotCentricCoords = rm.turnToRobotCentric(goal, currentPose, telemetry);

        double realPerpendicularEndDistance = robotCentricCoords.getY();
        double realParallelEndDistance = robotCentricCoords.getX();

//        Pose motorPowers = rm.goToPoint(robotCentricCoords,
//                                        currentPose,
//                                        realPerpendicularEndDistance,
//                                        realParallelEndDistance,
//                                        realThetaEndDistance);

//        Pose motorPowers = rm.goToPoint(goal, currentPose,
//                realPerpendicularEndDistance,
//                realParallelEndDistance,
//                realThetaEndDistance);

        RobotConstants.setUpperParallelPID(upperParallelPID);
        RobotConstants.setLowerParallelPID(lowerParallelPID);
        RobotConstants.setUpperPerpendicularPID(upperPerpendicularPID);
        RobotConstants.setLowerPerpendicularPID(lowerPerpendicularPID);
        RobotConstants.setUpperRotationalPID(upperRotationalPID);
        RobotConstants.setLowerRotationalPID(lowerRotationalPID);
        rm.updateControllerCoefficients();

//        robot.drive_update(new Pose(-robotCentricCoords.getY() * 0.2, robotCentricCoords.getX() * 0.2,
//                                    -motorPowers.getTheta()));

        FtcDashboard.getInstance().getTelemetry().addData("Target X", goal_x);
        FtcDashboard.getInstance().getTelemetry().addData("Target Y", goal_y);
        FtcDashboard.getInstance().getTelemetry().addData("Target Theta", goal_theta);
        FtcDashboard.getInstance().getTelemetry().addData("Actual X", currentPose.getX());
        FtcDashboard.getInstance().getTelemetry().addData("Actual Y", currentPose.getY());
        FtcDashboard.getInstance().getTelemetry().addData("Actual Theta", currentPose.getTheta());
        FtcDashboard.getInstance().getTelemetry().addData("Error Theta", realThetaEndDistance);
        FtcDashboard.getInstance().getTelemetry().update();

        telemetry.addData("X: ", currentPose.getX());
        telemetry.addData("Y: ", currentPose.getY());
        telemetry.addData("Theta: ", currentPose.getTheta());
//        telemetry.addData("Motor X: ", motorPowers.getX());
//        telemetry.addData("Motor Y: ", motorPowers.getY());
//        telemetry.addData("Motor Theta: ", motorPowers.getTheta());
        telemetry.update();
    }
}