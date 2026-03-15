package org.firstinspires.ftc.teamcode.PurePursuit.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;

@Disabled
@Autonomous(name = "Path Test", group = "Test")
public class TestOpMode extends OpMode {

    private RobotMovement robotMovement;
    private RobotMap robotMap;
    private Timer timer;

    /* -- States -- */
    private enum PathState {
        FIRST,
        SECOND,
        STOP,
        PATATA
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

//    private Pose[] second = {
//        new Pose(0,0,0),
//        new Pose(, 0,0),
//        new Pose(0, 40,270),
//        new Pose(0,0,270)
//    };

    /* -- Extra Util Functions -- */
    public void setPathState(PathState pState) {
        pathState = pState;
        timer.resetTimer();
    }

    /* -- Le program -- */
    @Override
    public void init() {
        robotMap = new RobotMap(hardwareMap, telemetry);
        robotMovement = new RobotMovement(robotMap, startingPose);
        timer = new Timer();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case FIRST:
                robotMovement.setThetaInterpolation(RobotMovement.ThetaInterpolation.HYBRID);
                robotMovement.setReversed(true);
                currentPath = first;
                robotMovement.reset();
//                setPathState(PathState.SECOND);
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
                if (timer.getElapsedTimeSeconds() >= 2) {
                    setPathState(PathState.PATATA);
                }
                break;
        }
    }

    @Override
    public void start() {
        setPathState(PathState.FIRST);
    }

    @Override
    public void loop() {
        autoPathUpdate();
        robotMovement.followPathUpdate(currentPath);

        /* -- Telemetry -- */
        telemetry.addData("Null: ", robotMovement.isNullDetected());
        telemetry.addData("Finished: ", robotMovement.isFinished());
        telemetry.addData("X: ", robotMovement.getCurrentPose().getX());
        telemetry.addData("Y: ", robotMovement.getCurrentPose().getY());
        telemetry.addData("Theta: ", robotMovement.getCurrentPose().getTheta());
        telemetry.update();
    }

    @Override
    public void stop() {
        Pose pose = robotMovement.getCurrentPose();
        // add pose transfer
    }
}