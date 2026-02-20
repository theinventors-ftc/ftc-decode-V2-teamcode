package org.firstinspires.ftc.teamcode.AutoOPs;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
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

import org.firstinspires.ftc.teamcode.DecodeRobot;
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

@Autonomous(name = "BLUE_12Ball", group = "Autonomous")
@Configurable
public class BLUE_12Ball extends CommandOpMode {
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
        follower.setStartingPose(new Pose(36.4, 135, Math.toRadians(180)));
        paths = new Paths(follower);

        intake = new Intake(robotMap);
        passthough = new Passthough(robotMap, MotifStorage.Motif.PPG);
        shooter = new Shooter(robotMap, this::getPoseFTCCoor, DecodeRobot.Alliance.BLUE, false);
        commandVault = new CommandSeriesVault(intake, passthough, shooter);

        commandVault.enableWheels().schedule();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        loopTime = new Timer();

        new SequentialCommandGroup(
                new FollowerCommand(follower, paths.StartToGoal),
                commandVault.autonomousWaitForTurret(),
                new InstantCommand(shooter::cacheCurrentDistance),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.GoalToIntakeStack1, 0.65),
                new WaitCommand(1000),
                new ParallelCommandGroup(
                        commandVault.rearrangeArtifacts(),
                        new FollowerCommand(follower, paths.Intake1ToLauchArea1)
                ),
                commandVault.stopIntakeProc(),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(200),
                new InstantCommand(shooter::cacheCurrentDistance),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.LaunchArea1ToIntakeStack2, 0.65),
                new WaitCommand(1000),
                new ParallelCommandGroup(
//                        commandVault.rearrangeArtifacts(),
                        new FollowerCommand(follower, paths.IntakeStack2ToLaunchArea2)
                ),
                commandVault.stopIntakeProc(),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(860),
                new InstantCommand(shooter::cacheCurrentDistance),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.LauchArea2ToIntakeStack3, 0.65),
                new WaitCommand(1000),
                new ParallelCommandGroup(
//                        commandVault.rearrangeArtifacts(),
                        new FollowerCommand(follower, paths.IntakeStack3ToSmallLaunchArea)
                ),
                commandVault.stopIntakeProc(),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(1000),
                new InstantCommand(shooter::cacheCurrentDistance),
                commandVault.feedAllFingers(),
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
                GoalToIntakeStack1,
                Intake1ToLauchArea1,
                IntakeStack2ToLaunchArea2,
                LaunchArea1ToIntakeStack2,
                LauchArea2ToIntakeStack3,
                IntakeStack3ToSmallLaunchArea,
                SmallLaunchAreaToParking;

        public Paths(Follower follower) {
            StartToGoal = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144.0-107.6, 135.0),
                                    new Pose(144.0-103.7, 120.7)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                    .build();

            GoalToIntakeStack1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144.0-103.7, 120.7),
                                    new Pose(144.0-84.5, 81.0),
                                    new Pose(144.0-93.6, 82.5),
                                    new Pose(144.0-123.0, 83.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                    .setBrakingStrength(deccel_strength)
                    .build();

            Intake1ToLauchArea1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144.0-123.0, 83.5),
                                    new Pose(144.0-86.0, 83.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(210.0))
                    .build();

            LaunchArea1ToIntakeStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144.0-86.0, 83.5),
                                    new Pose(144.0-75.0, 64),
                                    new Pose(144.0-133.7, 59.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(210.0), Math.toRadians(180.0))
                    .setBrakingStrength(deccel_strength)
                    .build();

            IntakeStack2ToLaunchArea2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144.0-133.7, 59.5),
                                    new Pose(144.0-107.0, 55.8),
                                    new Pose(144.0-84.0, 70.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(180.0))
                    .build();

            LauchArea2ToIntakeStack3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144.0-84.0, 70.5),
                                    new Pose(144.0-83.4, 36.1),
                                    new Pose(144.0-98.7, 35.075),
                                    new Pose(144.0-129.0, 35.6)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(170.0), Math.toRadians(180.0))
                    .setBrakingStrength(deccel_strength)
                    .build();

            IntakeStack3ToSmallLaunchArea = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144.0-129.0, 35.6),
                                    new Pose(144.0-90.0, 15.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(155.0))
                    .setTimeoutConstraint(600)
                    .build();

            SmallLaunchAreaToParking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144.0-90.0, 15.5),
                                    new Pose(144.0-112.0, 13.0)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(155.0), Math.toRadians(180.0))
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