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
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FollowerCommand;

import java.util.ArrayList;

@Autonomous(name = "RED_Allianca_Automatrone", group = "Autonomous")
@Configurable
public class RED_Allianca_Automatrone extends CommandOpMode {
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
        follower.setStartingPose(new Pose(107.6, 135, Math.toRadians(180)));
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
                new InstantCommand(shooter::cacheCurrentDistance),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.GoalToIntakeStack2),
                new FollowerCommand(follower, paths.GoalToIntakeStack2_Deccel, 0.7),
                new WaitCommand(1000),
                new ParallelCommandGroup(
                        commandVault.rearrangeArtifacts(),
                        new FollowerCommand(follower, paths.IntakeStack2ToLaunchArea2)
                ),
                commandVault.stopIntakeProc(),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(200),
                new InstantCommand(shooter::cacheCurrentDistance),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.LaunchArea2ToGate, 0.7),
                new FollowerCommand(follower, paths.GateCross),
                new WaitCommand(1500),
                new ParallelCommandGroup(
                        commandVault.rearrangeArtifacts(),
                        new FollowerCommand(follower, paths.GateToLaunch)
                ),
                commandVault.stopIntakeProc(),

                commandVault.autonomousWaitForTurret(),
                new WaitCommand(860),
                new InstantCommand(shooter::cacheCurrentDistance),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.LaunchToStack1, 0.65),
                new WaitCommand(1000),
                new ParallelCommandGroup(
                        commandVault.rearrangeArtifacts(),
                        new FollowerCommand(follower, paths.Stack1ToLaunch)
                ),
                commandVault.stopIntakeProc(),

                commandVault.autonomousWaitForTurret(),
                new WaitCommand(1000),
                new InstantCommand(shooter::cacheCurrentDistance),
                commandVault.feedAllFingers(),
                new FollowerCommand(follower, paths.Parking)
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
                GoalToIntakeStack2,
                IntakeStack2ToLaunchArea2,
                GoalToIntakeStack2_Deccel,
                LaunchArea2ToGate,
                GateCross,
                GateToLaunch,
                LaunchToStack1,
                Stack1ToLaunch,
                Parking;

        public Paths(Follower follower) {
            StartToGoal = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(107.6, 135.0),
                                    new Pose(103.7, 120.7)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(260))
                    .build();

            GoalToIntakeStack2 = follower.pathBuilder().addPath(
                    new BezierCurve(
                            new Pose(103.7, 120.7),
                            new Pose(85.0, 80),
                            new Pose(90, 60.5)
                    )
            ).setLinearHeadingInterpolation(Math.toRadians(260), Math.toRadians(0))
            .build();

            GoalToIntakeStack2_Deccel = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(90, 60.5),
                                    new Pose(95, 60),
                                    new Pose(129, 60.5)
                            )
                    ).setConstantHeadingInterpolation(0)
                    .setBrakingStrength(deccel_strength)
                    .build();

            IntakeStack2ToLaunchArea2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(129, 60.5),
                                    new Pose(107.0, 55.8),
                                    new Pose(84.0, 74.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                    .build();

            LaunchArea2ToGate = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.0, 74.5),
                                    new Pose(95.0, 65),
                                    new Pose(123, 66)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(0))
                    .setBrakingStrength(deccel_strength)
                    .build();

            GateCross = follower.pathBuilder().addPath(
                new BezierCurve(
                        new Pose(123, 66),
                        new Pose(110, 55),
                        new Pose(128, 60)
                )
            ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(27))
            .build();

            GateToLaunch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128, 60),
                                    new Pose(84.0, 74.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(27), Math.toRadians(0))
                    .setTimeoutConstraint(300)
                    .build();

            LaunchToStack1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(84.0, 74.5),
                                    new Pose(72, 87),
                                    new Pose(123.0, 83.5)
                            )
                    ).setConstantHeadingInterpolation(0)
                    .setBrakingStrength(deccel_strength)
                    .build();

            Stack1ToLaunch = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(123.0, 83.5),
                                new Pose(84.0, 74.5)
                        )
                ).setConstantHeadingInterpolation(0)
                .setTimeoutConstraint(300)
                .build();

            Parking = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(84.0, 74.5),
                            new Pose(84.0, 50)
                    )
            ).setConstantHeadingInterpolation(0)
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