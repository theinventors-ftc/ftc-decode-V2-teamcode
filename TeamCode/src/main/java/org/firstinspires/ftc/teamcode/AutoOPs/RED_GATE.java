package org.firstinspires.ftc.teamcode.AutoOPs;

import static org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction.subtractPoses;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
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
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.Localization.NanoTimer;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FollowerCommand;

@Autonomous(name = "RED_GATE", group = "Autonomous")
@Configurable
public class RED_GATE extends CommandOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private RobotMap robotMap;

    private Intake intake;
    private Passthough passthough;
    private Shooter shooter;

    private CommandSeriesVault commandVault;

    private Timer loopTime;

    private Paths paths;
    private Pose pinpointPose;
    private NanoTimer timer;
    private long deltaTimeNano;

    @Override
    public void initialize() {
        pinpointPose = new Pose(117.5, 130.5, Math.toRadians(225));
        timer = new NanoTimer();
        deltaTimeNano = 1;
        CommandScheduler.getInstance().reset(); // Ultra SOS
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        robotMap = new RobotMap(hardwareMap, telemetry,null,null);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(117.5, 130.5, Math.toRadians(225)));
        paths = new Paths(follower);

        intake = new Intake(robotMap);
        passthough = new Passthough(robotMap, MotifStorage.Motif.PPG);
        shooter = new Shooter(robotMap, this::getPoseFTCCoor, this::getVelPoseFTCCoor,
                              DecodeRobotV2.Alliance.RED, false, true,
                              true);
        commandVault = new CommandSeriesVault(intake, passthough, shooter);

        commandVault.enableWheels().schedule();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        loopTime = new Timer();

        new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new FollowerCommand(follower, paths.StartToStack2, 0.6),
                    new SequentialCommandGroup(
                        commandVault.autonomousWaitForTurret(),
                        commandVault.feedAllFingers(),
                        commandVault.startIntakeProc())
                )
//                new FollowerCommand(follower, paths.Stack2ToShoot, 0.89),
//                new InstantCommand(follower::resumePathFollowing),
//                new WaitCommand(550),
//                new FollowerCommand(follower, paths.ShootToGate, 1),
//                commandVault.stopIntakeProc(),
//                new WaitCommand(160),
//                new FollowerCommand(follower, paths.GateToShoot),
//                commandVault.autonomousWaitForTurret(),
//
//                new WaitCommand(150),
//                commandVault.feedAllFingers(),
//                commandVault.startIntakeProc(),
//                new FollowerCommand(follower, paths.ShootToStack1, 1),
//                new WaitCommand(100),
//                new FollowerCommand(follower, paths.Stack1ToShoot),
//                commandVault.stopIntakeProc(),
//                commandVault.autonomousWaitForTurret(),
//                new WaitCommand(100),
//
//                commandVault.feedAllFingers(),
//                commandVault.startIntakeProc(),
//                new FollowerCommand(follower, paths., 0.95, true),
//                new InstantCommand(follower::resumePathFollowing),
//                new WaitCommand(250),
//                new ParallelCommandGroup(
//                        new FollowerCommand(follower, paths.IntakeStack3ToSmallLaunchArea),
//                        new SequentialCommandGroup(
//                                new WaitCommand(320),
//                                commandVault.reverseIntake()
//                        )
//                ),
//                commandVault.autonomousWaitForTurret(),
//                commandVault.stopIntakeProc(),
//                new WaitCommand(200),
////                new InstantCommand(shooter::cacheCurrentDistance),
//                commandVault.feedAllFingers(),
//                commandVault.startIntakeProc(),
//                new FollowerCommand(follower, paths.SmallLaunchAreaToHP),
//                new WaitCommand(400),
//                new ParallelCommandGroup(
//                        new FollowerCommand(follower, paths.HPToSmallLaunchArea),
//                        new SequentialCommandGroup(
//                                new WaitCommand(150),
//                                commandVault.reverseIntake()
//                        )
//                ),
//                commandVault.autonomousWaitForTurret(),
//                commandVault.stopIntakeProc(),
//                new WaitCommand(250),
////                new InstantCommand(shooter::cacheCurrentDistance),
//                commandVault.feedAllFingers(),
//                commandVault.parkShooter(),
//                new FollowerCommand(follower, paths.Park)
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
//        ArrayList<Double> dists = shooter.getCachedDistances();
//        for (int i = 0; i < dists.size(); i++) {
//            telemetry.addData("Dist " + i, dists.get(i));
//        }
//        telemetry.addData("Dists", shooter.getCachedDistances());
        telemetry.update();
    }

    public static class Paths {

        private final double deccel_strength = 0;

        public PathChain
                StartToStack2,
                Stack2ToShoot,
                ShootToGate,
                GateToShoot,
                ShootToStack1,
                Stack1ToShoot,
                Park;

        public Paths(Follower follower) {
            StartToStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(117.5, 130.5),
                                    new Pose(50.3, 74.2),
                                    new Pose(81.2, 54.8),
                                    new Pose(94.4, 57.9),
                                    new Pose(134.0, 58)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            Stack2ToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.0, 58),
                                    new Pose(99, 58),
                                    new Pose(88, 75)
                            )
                    ).setConstantHeadingInterpolation(0)
                    .setBrakingStrength(1.3)
                    .build();

            ShootToGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88, 75),
                                    new Pose(130, 60.57)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .setBrakingStrength(0.4)
                    .build();

            GateToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130, 60.57),
                                    new Pose(88, 75)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();

            ShootToStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88, 75),
                                    new Pose(125.5, 83.5)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setBrakingStrength(1.9)
                    .build();

            Stack1ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.5, 83.5),
                                    new Pose(88, 83.5)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Park = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88, 83.5),
                                    new Pose(88, 60)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
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

    public org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose getVelPoseFTCCoor() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();

        Pose deltaPose = follower.getPose().minus(pinpointPose);
        pinpointPose = follower.getPose();

        return new org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose(
            deltaPose.getY() / (deltaTimeNano / Math.pow(10.0, 9)),
            deltaPose.getX() / (deltaTimeNano / Math.pow(10.0, 9)),
            0
        );
    }

    @Override
    public void reset() {
        super.reset();
        PoseStorage.currentPose = getPoseFTCCoor();
    }
}