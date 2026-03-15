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

import org.firstinspires.ftc.teamcode.DecodeRobotV2;
import org.firstinspires.ftc.teamcode.Mechanisms.CommandSeriesVault;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Passthough;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.MotifStorage;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FollowerCommand;

import java.util.ArrayList;

@Autonomous(name = "RED_15_Ball_HP", group = "Autonomous")
@Configurable
public class RED_15_Ball_HP extends CommandOpMode {
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
        follower.setStartingPose(new Pose(120 - 5.625, 128.89763779, Math.toRadians(0)));
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
//                new InstantCommand(shooter::cacheCurrentDistance),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.GoalToIntakeStack2, 0.9, true),
                new InstantCommand(follower::resumePathFollowing),
                new WaitCommand(550),
                new FollowerCommand(follower, paths.IntakeStack2ToOpenGate, 1),
                commandVault.stopIntakeProc(),
                new WaitCommand(160),
                new FollowerCommand(follower, paths.OpenGate2ToLaunchArea2),
                commandVault.autonomousWaitForTurret(),
//                new InstantCommand(shooter::cacheCurrentDistance),
                new WaitCommand(150),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.LauchArea2ToIntakeStack1, 1),
                new WaitCommand(100),
                new FollowerCommand(follower, paths.Intake1ToLauchArea1),
                commandVault.stopIntakeProc(),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(100),
//                new InstantCommand(shooter::cacheCurrentDistance),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.LauchArea1ToIntakeStack3, 0.95, true),
                new InstantCommand(follower::resumePathFollowing),
                new WaitCommand(250),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.IntakeStack3ToSmallLaunchArea),
                        new SequentialCommandGroup(
                                new WaitCommand(320),
                                commandVault.reverseIntake()
                        )
                ),
                commandVault.autonomousWaitForTurret(),
                commandVault.stopIntakeProc(),
                new WaitCommand(200),
//                new InstantCommand(shooter::cacheCurrentDistance),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.SmallLaunchAreaToHP),
                new WaitCommand(400),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.HPToSmallLaunchArea),
                        new SequentialCommandGroup(
                                new WaitCommand(150),
                                commandVault.reverseIntake()
                        )
                ),
                commandVault.autonomousWaitForTurret(),
                commandVault.stopIntakeProc(),
                new WaitCommand(250),
//                new InstantCommand(shooter::cacheCurrentDistance),
                commandVault.feedAllFingers(),
                commandVault.parkShooter(),
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

        private final double deccel_strength = 0;

        public PathChain
                StartToGoal,
                GoalToIntakeStack2,
                IntakeStack2ToOpenGate,
                OpenGate2ToLaunchArea2,
                LauchArea2ToIntakeStack1,
                Intake1ToLauchArea1,
                LauchArea1ToIntakeStack3,
                IntakeStack3ToSmallLaunchArea,
                SmallLaunchAreaToHP,
                HPToSmallLaunchArea,
                SmallLaunchAreaToParking;

        public Paths(Follower follower) {
            StartToGoal = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(108.0, 135.0),
                                    new Pose(91, 102.0)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(260))
                    .build();

            GoalToIntakeStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(91, 102.0),
                                    new Pose(75.5, 70),
                                    new Pose(93.0, 52.0),
                                    new Pose(136.8, 60)
                            )
                    ).setConstantHeadingInterpolation(0)
                    .setBrakingStrength(1.3)
                    .build();

            IntakeStack2ToOpenGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(136.8, 60),
                                    new Pose(112.0, 60),
                                    new Pose(126, 70)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                    .setBrakingStrength(deccel_strength)
                    .build();

            OpenGate2ToLaunchArea2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(126, 70),
                                    new Pose(95, 68),
                                    new Pose(87.5, 83.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                    .build();

            LauchArea2ToIntakeStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84, 83.5),
                                    new Pose(125.5, 83.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setBrakingStrength(1.9)
                    .build();

            Intake1ToLauchArea1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(129.5, 83.5),
                                    new Pose(86.0, 83.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                    .build();

            LauchArea1ToIntakeStack3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86.0, 83.5),
                                    new Pose(80.0, 34),
                                    new Pose(75.0, 35.6),
                                    new Pose(141.5, 35.6)
                            )
                    ).setConstantHeadingInterpolation(0)
                    .setBrakingStrength(1.2)
                    .build();

            IntakeStack3ToSmallLaunchArea = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(141.5, 35.6),
                                    new Pose(90.0, 32.0),
                                    new Pose(102, 14.5)
                            )
                    ).setConstantHeadingInterpolation(0)
                    .build();

            SmallLaunchAreaToHP = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(102, 14.5),
                                    new Pose(120, 24.0),
                                    new Pose(130.0, 14.0)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(328.0))
                    .build();

            HPToSmallLaunchArea = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(130.0, 14.0),
                                    new Pose(120.0, 16.0),
                                    new Pose(102, 14.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(328.0), Math.toRadians(0))
                    .build();

            SmallLaunchAreaToParking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102, 14.5),
                                    new Pose(104.0, 16.0)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
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

    @Override
    public void reset() {
        super.reset();
        PoseStorage.currentPose = getPoseFTCCoor();
    }
}