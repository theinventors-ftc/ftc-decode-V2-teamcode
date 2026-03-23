package org.firstinspires.ftc.teamcode.PurePursuit.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DecodeRobotV2;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Util.Timer;

@Autonomous(name = "Test_Pink", group = "Test")
public class TestOpMode extends CommandOpMode {
    private DriveConstants driveConstants;
    private DecodeRobotV2 robot;
    private RobotMovement robotMovement;
    private RobotMap robotMap;
    private Timer timer;

    /* -- States -- */
    private enum PathState {
        FIRST,
        SECOND,
        STOP,
        patata
    }
    private PathState pathState;

    /* -- Poses and Vectors -- */
    private Pose startingPose = new Pose(0, 0, 0);

    /* -- Paths -- */
    private Pose[] currentPath;

    private Pose[] first = {
        startingPose,
        new Pose(30, 0,0)
    };

    private Pose[] second = {
        new Pose(0,0,0),
        new Pose(20, 0,0),
        new Pose(0, 40,270),
        new Pose(0,0,270)
    };

    /* -- Extra Util Functions -- */
    public void setPathState(PathState pState) {
        pathState = pState;
        timer.resetTimer();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case FIRST:
                robotMovement.setType(RobotMovement.Type.ENGAGED);
                robotMovement.setThetaInterpolation(
                    RobotMovement.ThetaInterpolation.HYBRID
                );
                currentPath = first;
                robotMovement.reset();
                setPathState(PathState.STOP);
                break;

//            case SECOND:
//                if (robotMovement.getRealTranslationalEndDistance() <= 4) {
//                    Pose p = robotMovement.getCurrentPose();
//                    second[0] = new Pose(p.getX(), p.getY(), p.getTheta());
//                    currentPath = second;
//                    robotMovement.reset();
//
//                    RobotConstants.setMinRadiusRange(18);
//                    RobotConstants.setMaxRadiusRange(20);
//                    setPathState(PathState.STOP);
//                }
//                break;

            case STOP:
                if (timer.getElapsedTimeSeconds() >= 10) {
                    setPathState(PathState.patata);
                }
                break;
        }
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robotMap = new RobotMap(hardwareMap, telemetry);
        robotMovement = new RobotMovement(robotMap, startingPose);
        timer = new Timer();

        /*-- Drive Constants --*/
        driveConstants = new DriveConstants();

        driveConstants.frontLeftInverted = true;
        driveConstants.frontRightInverted = false;
        driveConstants.rearRightInverted = false;
        driveConstants.rearLeftInverted = true;

        driveConstants.DEFAULT_SPEED_PERC = 1.0;
        driveConstants.SLOW_SPEED_PERC = 0.7;

        //- Default Path
        setPathState(PathState.FIRST);

        robot = new DecodeRobotV2(
            robotMap,
            driveConstants,
            DecodeRobotV2.Alliance.RED
        );

        robot.setAutoEnabled(true);
    }

    @Override
    public void run() {
        super.run();
        autoPathUpdate();
        robotMovement.followPathUpdate(currentPath);
        robot.drive_update(robotMovement.getPowers());
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }
}