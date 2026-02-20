package org.firstinspires.ftc.teamcode.AutoOPs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.DecodeRobot;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Util.Timer;

@Disabled
@Autonomous(name = "Do not run this AutoOp", group = "")
public class AutoOpBase extends CommandOpMode {
    private DriveConstants driveConstants;
    private DecodeRobot robot;
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
    private Pose startingPose = new Pose(0, 0, 180);

    /* -- Paths -- */
    private Pose[] currentPath;

    private Pose[] first = {
        startingPose,
        new Pose(50, 0,0),
        new Pose(50, -80,0),
        new Pose(100, -80,0),
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
                robotMovement.setThetaInterpolation(
                    RobotMovement.ThetaInterpolation.HYBRID
                );
                robotMovement.setReversed(true);
                currentPath = first;
                robotMovement.reset();
                setPathState(PathState.SECOND);
                break;

            case SECOND:
                if (robotMovement.getRealTranslationalEndDistance() <= 4) {
                    Pose p = robotMovement.getCurrentPose();
                    second[0] = new Pose(p.getX(), p.getY(), p.getTheta());
                    currentPath = second;
                    robotMovement.reset();

                    RobotConstants.setMinRadiusRange(18);
                    RobotConstants.setMaxRadiusRange(20);
                    setPathState(PathState.STOP);
                }
                break;

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
        driveConstants.frontRightInverted = true;
        driveConstants.rearRightInverted = true;
        driveConstants.rearLeftInverted = true;

        driveConstants.DEFAULT_SPEED_PERC = 1.0;
        driveConstants.SLOW_SPEED_PERC = 0.7;

        //- Default Path
        setPathState(PathState.FIRST);
        robot.setAutoEnabled(true);
    }

    public void initAllianceRelated(DecodeRobot.Alliance alliance) {
        robot = new DecodeRobot(
            robotMap,
            driveConstants,
            alliance
        );
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
