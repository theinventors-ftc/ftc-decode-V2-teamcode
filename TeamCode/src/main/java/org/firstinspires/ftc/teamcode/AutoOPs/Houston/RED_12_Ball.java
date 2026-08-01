package org.firstinspires.ftc.teamcode.AutoOPs.Houston;

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
import org.firstinspires.ftc.teamcode.Mechanisms.Detection;
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

@Autonomous(name = "RED_12_Ball", group = "Autonomous")
@Configurable
public class RED_12_Ball extends CommandOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private RobotMap robotMap;

    private Detection detection;

    private Intake intake;
    private Passthough passthough;
    private Shooter shooter;

    private CommandSeriesVault commandVault;

    private Timer loopTime;
    private Pose pinpointPose;
    private NanoTimer timer;
    private long deltaTimeNano;

    private Paths paths;

    @Override
    public void initialize() {
        pinpointPose = new Pose(117.5, 129.3, Math.toRadians(46));
        timer = new NanoTimer();
        deltaTimeNano = 1;
        CommandScheduler.getInstance().reset(); // Ultra SOS
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        robotMap = new RobotMap(hardwareMap, telemetry,null,null);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(117, 129.3, Math.toRadians(46)));
        paths = new Paths(follower);

        intake = new Intake(robotMap);
        passthough = new Passthough(robotMap, MotifStorage.Motif.PPG);
        shooter = new Shooter(robotMap, this::getPoseFTCCoor, this::getVelPoseFTCCoor,
                this::getAccelPoseFTCCoor, DecodeRobotV2.Alliance.RED,
                false, false, true);
        detection = new Detection(robotMap);
        commandVault = new CommandSeriesVault(intake, passthough, shooter, detection);

        commandVault.enableWheels().schedule();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        loopTime = new Timer();

        new SequentialCommandGroup(
                new FollowerCommand(follower, paths.StartToGoal),
                commandVault.autonomousWaitForTurret(),
                commandVault.feedAllHingesFingersAUTO(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.GoalToIntakeStack2, 0.9, true),
                new InstantCommand(follower::resumePathFollowing),
                new WaitCommand(550),
                new FollowerCommand(follower, paths.IntakeStack2ToOpenGate, 1),
                commandVault.stopIntakeProc(),
                new WaitCommand(160),
                new FollowerCommand(follower, paths.OpenGate2ToLaunchArea2),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(150),
                commandVault.feedAllHingesFingersAUTO(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.LauchArea2ToIntakeStack1, 1),
                new WaitCommand(100),
                new FollowerCommand(follower, paths.Intake1ToLauchArea1),
                commandVault.stopIntakeProc(),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(100),
                commandVault.feedAllHingesFingersAUTO(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.LauchArea1ToIntakeStack3, 0.95, true),
                new InstantCommand(follower::resumePathFollowing),
                new WaitCommand(250),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.IntakeStack3ToShoot3),
                        new SequentialCommandGroup(
                                new WaitCommand(320),
                                commandVault.reverseIntake()
                        )
                ),
                commandVault.autonomousWaitForTurret(),
                commandVault.stopIntakeProc(),
                new WaitCommand(200),
                commandVault.feedAllHingesFingersAUTO(),
                commandVault.parkShooter(),
                new FollowerCommand(follower, paths.Shoot3ToParking)
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
                IntakeStack3ToShoot3,
                Shoot3ToParking;

        public Paths(Follower follower) {
            StartToGoal = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(117, 129.3),
                                    new Pose(91, 102.0)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(46))
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
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
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

            IntakeStack3ToShoot3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(141.5, 35.6),
                                    new Pose(95, 68),
                                    new Pose(87.5, 83.5)
                            )
                    ).setConstantHeadingInterpolation(0)
                    .build();

            Shoot3ToParking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.5, 83.5),
                                    new Pose(87.5, 60)
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

    public org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose getAccelPoseFTCCoor() {

        return new org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose(
                follower.getAcceleration().getYComponent(),
                follower.getAcceleration().getXComponent(),
                0
        );
    }

    @Override
    public void reset() {
        super.reset();
        PoseStorage.currentPose = getPoseFTCCoor();
    }
}