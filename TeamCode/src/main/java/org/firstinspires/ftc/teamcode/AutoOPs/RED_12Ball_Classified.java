package org.firstinspires.ftc.teamcode.AutoOPs;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.DecodeRobotV2;
import org.firstinspires.ftc.teamcode.Mechanisms.CommandSeriesVault;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Passthough;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.MotifStorage;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FollowerCommand;

import java.util.ArrayList;

@Autonomous(name = "RED_12Ball_Classified", group = "Autonomous")
@Configurable
public class RED_12Ball_Classified extends CommandOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private RobotMap robotMap;

    private Intake intake;
    private Passthough passthough;
    private Shooter shooter;

    private CommandSeriesVault commandVault;

    private Timer loopTime;

    private Paths paths;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset(); // Ultra SOS
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        robotMap = new RobotMap(hardwareMap, telemetry,null,null);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(107.6, 135, Math.toRadians(0)));
        paths = new Paths(follower);

        intake = new Intake(robotMap);
        passthough = new Passthough(robotMap, MotifStorage.Motif.PPG);
        shooter = new Shooter(robotMap, this::getPoseFTCCoor, DecodeRobotV2.Alliance.RED, false);
        commandVault = new CommandSeriesVault(intake, passthough, shooter);

        commandVault.enableWheels().schedule();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        loopTime = new Timer();

        new SequentialCommandGroup(
                new FollowerCommand(follower, paths.StartToGoal),
                commandVault.autonomousWaitForTurret(),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.GoalToMidPoint, 1, true),
                new FollowerCommand(follower, paths.MidPointToIntakeStack2, 0.4),
                new InstantCommand(follower::resumePathFollowing),
                new WaitCommand(1000),
                new FollowerCommand(follower, paths.IntakeStack2ToOpenGate),
                commandVault.stopIntakeProc(),
                new FollowerCommand(follower, paths.OpenGate2ToLaunchArea2),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(150),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.LauchArea2ToIntakeStack1, 0.4),
                new WaitCommand(1000),
                new FollowerCommand(follower, paths.Intake1ToLauchArea1),
                commandVault.stopIntakeProc(),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(150),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.LauchArea1ToMidPoint, 1, true),
                new FollowerCommand(follower, paths.MidPointToIntakeStack3, 0.4),
                new InstantCommand(follower::resumePathFollowing),
                new WaitCommand(1000),
                new FollowerCommand(follower, paths.IntakeStack3ToSmallLaunchArea),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(250),
                commandVault.feedAllFingers(),
                commandVault.stopIntakeProc(),
                new FollowerCommand(follower, paths.SmallLaunchAreaToParking)
        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        for (LynxModule hub : robotMap.getHubs()) hub.clearBulkCache();

        telemetry.addData("Loop Hz: ", 1.0/loopTime.getElapsedTimeSeconds());
        loopTime.resetTimer();

        telemetry.addData("X", getPoseFTCCoor().getX());
        telemetry.addData("Y", getPoseFTCCoor().getY());
        telemetry.addData("Heading", getPoseFTCCoor().getTheta());
        ArrayList<Double> dists = shooter.getCachedDistances();
        for (int i = 0; i < dists.size(); i++) {
            telemetry.addData("Dist " + i, dists.get(i));
        }
        telemetry.addData("Dists", shooter.getCachedDistances());
        telemetry.update();
    }


    public static class Paths {

        private final double deccel_strength = 0.5;
        public PathChain
                StartToGoal,
                GoalToMidPoint,
                MidPointToIntakeStack2,
                IntakeStack2ToOpenGate,
                OpenGate2ToLaunchArea2,
                LauchArea2ToIntakeStack1,
                Intake1ToLauchArea1,
                LauchArea1ToMidPoint,
                MidPointToIntakeStack3,
                IntakeStack3ToSmallLaunchArea,
                SmallLaunchAreaToParking;

        public Paths(Follower follower) {
            StartToGoal = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(107.6, 135.0),
                                    new Pose(87.7, 105.7)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(260))
                    .build();

            GoalToMidPoint = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.7, 105.7),
                                    new Pose(75.5, 70),
                                    new Pose(70.0, 54),
                                    new Pose(105, 59.5)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            MidPointToIntakeStack2 = follower.pathBuilder().addPath(
                    new BezierLine(
                        new Pose(105, 59.5),
                        new Pose(133.7, 59.5)
                    )
            ).setConstantHeadingInterpolation(0)
            .setBrakingStrength(deccel_strength)
            .build();

            IntakeStack2ToOpenGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(133.7, 59.5),
                                    new Pose(115.0, 60),
                                    new Pose(125.5, 65)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                    .setBrakingStrength(deccel_strength)
                    .build();

            OpenGate2ToLaunchArea2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(125, 68),
                                    new Pose(95, 68),
                                    new Pose(84.0, 83.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                    .build();

            LauchArea2ToIntakeStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.0, 83.5),
                                    new Pose(123.0, 83.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setBrakingStrength(deccel_strength)
                    .build();

            Intake1ToLauchArea1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.0, 83.5),
                                    new Pose(86.0, 83.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                    .build();

            LauchArea1ToMidPoint = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.0, 83.5),
                                    new Pose(83.4, 36.1),
                                    new Pose(98.7, 34),
                                    new Pose(85.0, 35.6)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(30))
                    .build();

            MidPointToIntakeStack3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(85.0, 35.6),
                                    new Pose(129.0, 35.6)
                            )
            ).setConstantHeadingInterpolation(0)
            .setBrakingStrength(deccel_strength)
            .build();

            IntakeStack3ToSmallLaunchArea = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.0, 35.6),
                                    new Pose(90.0, 15.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))
                    .setTimeoutConstraint(600)
                    .build();

            SmallLaunchAreaToParking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.0, 15.5),
                                    new Pose(100.0, 13.0)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                    .build();
        }
    }

    public org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose getPoseFTCCoor() {
        Pose pedroPose = new Pose(
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading()
        ).getAsCoordinateSystem(FTCCoordinates.INSTANCE);

        return new org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose(
                pedroPose.getX(),
                pedroPose.getY(),
                Math.toDegrees(pedroPose.getHeading())
        );
    }
}