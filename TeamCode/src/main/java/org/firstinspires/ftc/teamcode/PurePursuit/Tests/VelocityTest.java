package org.firstinspires.ftc.teamcode.PurePursuit.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DecodeRobotV2;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.RobotMap;

@Config
@Autonomous(name = "Velocity Test", group = "Test")
public class VelocityTest extends CommandOpMode {

    private DriveConstants driveConstants;
    private DecodeRobotV2 robot;

    private RobotMap robotMap;
    private RobotMovement rm;

    public static double goal_x = 0, goal_y = 0, goal_theta = 0;
    public static double maxVelPar = 0, maxVelPerp = 0, maxVelTheta = 0;

    private Pose startingPose = new Pose(0, 0, 0);
    private Pose goal = new Pose(goal_x,goal_y,goal_theta);

    @Override
    public void initialize () {
        CommandScheduler.getInstance().reset(); // Ultra SOS
        robotMap = new RobotMap(hardwareMap, telemetry);
        rm = new RobotMovement(robotMap, startingPose);

        /*-- Drive Constants --*/
        driveConstants = new DriveConstants();

        driveConstants.frontLeftInverted = true;
        driveConstants.frontRightInverted = false;
        driveConstants.rearRightInverted = false;
        driveConstants.rearLeftInverted = true;

        driveConstants.DEFAULT_SPEED_PERC = 1.0;
        driveConstants.SLOW_SPEED_PERC = 0.7;

        robot = new DecodeRobotV2(
            robotMap, driveConstants,
            DecodeRobotV2.Alliance.RED
        );

        robot.setAutoEnabled(true);
    }

    @Override
    public void run () {
        goal = new Pose(goal_x, goal_y, goal_theta);

        super.run();
        for (LynxModule hub : robotMap.getHubs()) hub.clearBulkCache();

        rm.updateLocalizer();
        Pose currentPose = rm.getCurrentPose();

        double realPerpendicularEndDistance = goal.getY() - currentPose.getY();
        double realParallelEndDistance = goal.getX() - currentPose.getX();
        double realThetaEndDistance = goal.getTheta() - currentPose.getTheta();

        robot.drive_update(goal);

        maxVelPar = Math.max(rm.getCurrentVelocity().getX(), maxVelPar);
        maxVelPerp = Math.max(rm.getCurrentVelocity().getY(), maxVelPerp);
        maxVelTheta = Math.max(rm.getCurrentVelocity().getTheta(), maxVelTheta);

        FtcDashboard.getInstance().getTelemetry().addData("Target X", goal_x);
        FtcDashboard.getInstance().getTelemetry().addData("Target Y", goal_y);
        FtcDashboard.getInstance().getTelemetry().addData("Target Theta", goal_theta);
        FtcDashboard.getInstance().getTelemetry().addData("Actual X", rm.getCurrentVelocity().getX());
        FtcDashboard.getInstance().getTelemetry().addData("Actual Y", rm.getCurrentVelocity().getY());
        FtcDashboard.getInstance().getTelemetry().addData("Actual Theta", rm.getCurrentVelocity().getTheta());
        FtcDashboard.getInstance().getTelemetry().addData("Max X", maxVelPar);
        FtcDashboard.getInstance().getTelemetry().addData("Max Y", maxVelPerp);
        FtcDashboard.getInstance().getTelemetry().addData("Max Theta", maxVelTheta);
        FtcDashboard.getInstance().getTelemetry().update();

        telemetry.addData("X: ", rm.getCurrentVelocity().getX());
        telemetry.addData("Y: ", rm.getCurrentVelocity().getY());
        telemetry.addData("Theta: ", rm.getCurrentVelocity().getTheta());
        telemetry.addData("Motor X: ", goal.getX());
        telemetry.addData("Motor Y: ", goal.getY());
        telemetry.addData("Motor Theta: ", goal.getTheta());
        telemetry.update();
    }
}
