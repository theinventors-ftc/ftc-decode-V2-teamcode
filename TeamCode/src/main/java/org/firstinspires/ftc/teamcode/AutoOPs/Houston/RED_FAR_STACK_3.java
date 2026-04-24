package org.firstinspires.ftc.teamcode.AutoOPs.Houston;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
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
import com.pedropathing.paths.HeadingInterpolator;
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
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FollowerCommand;

@Autonomous(name = "RED_FAR_STACK_3_Straight", group = "Autonomous")
@Configurable
public class RED_FAR_STACK_3 extends CommandOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private RobotMap robotMap;

    private Detection detection;

    private Intake intake;
    private Passthough passthough;
    private Shooter shooter;

    private CommandSeriesVault commandVault;

    private Timer loopTime, elapsedTime;

    private Paths paths;
    private long safeTime = 2700, wait_HP = 320;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset(); // Ultra SOS
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        robotMap = new RobotMap(hardwareMap, telemetry,null,null);

        elapsedTime = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(87.3, 8.95, Math.toRadians(90)));
        paths = new Paths(follower);

        intake = new Intake(robotMap);
        passthough = new Passthough(robotMap, MotifStorage.Motif.PPG);
        shooter = new Shooter(
                robotMap,
                this::getPoseFTCCoor,
                () -> new org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose(0, 0, 0),
                () -> new org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose(0, 0, 0),
                DecodeRobotV2.Alliance.RED,
                false,
                false,
                true
        );
        detection = new Detection(robotMap);
        commandVault = new CommandSeriesVault(intake, passthough, shooter, detection);

        commandVault.enableWheels().schedule();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        loopTime = new Timer();

        new SequentialCommandGroup(
                new InstantCommand(() -> elapsedTime.resetTimer()),
                commandVault.autonomousWaitForTurret(),
                commandVault.feedAllHingesFingersAUTO(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.StartToStack, 1),
                new WaitCommand(300),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                commandVault.stopIntakeProc()
                        ),
                        new FollowerCommand(follower, paths.StackToShoot,1)
                ),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(100),
                commandVault.feedAllHingesFingersAUTO(),

                // Repeated Part
                //
                commandVault.startIntakeProc(),
                new ParallelRaceGroup(
                    new FollowerCommand(follower, paths.ShootToHP, 1),
                    new WaitCommand(safeTime)
                ),
                new InstantCommand(follower::resumePathFollowing),
                new WaitCommand(wait_HP),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.HPToShoot,1),
                        new SequentialCommandGroup(
                                commandVault.reverseIntake(),
                                new WaitCommand(500),
                                commandVault.stopIntakeProc()
                        )
                ),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(100),
                commandVault.feedAllHingesFingersAUTO(),

                //
                commandVault.startIntakeProc(),
                new ParallelRaceGroup(
                    new FollowerCommand(follower, paths.ShootToHP, 1),
                    new WaitCommand(safeTime)
                ),
                new InstantCommand(follower::resumePathFollowing),
                new WaitCommand(wait_HP),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.HPToShoot,1),
                        new SequentialCommandGroup(
                                commandVault.reverseIntake(),
                                new WaitCommand(500),
                                commandVault.stopIntakeProc()
                        )
                ),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(100),
                commandVault.feedAllHingesFingersAUTO(),

                //
                commandVault.startIntakeProc(),
                new ParallelRaceGroup(
                    new FollowerCommand(follower, paths.ShootToHP, 1),
                    new WaitCommand(safeTime)
                ),
                new InstantCommand(follower::resumePathFollowing),
                new WaitCommand(wait_HP),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.HPToShoot,1),
                        new SequentialCommandGroup(
                                commandVault.reverseIntake(),
                                new WaitCommand(500),
                                commandVault.stopIntakeProc()
                        )
                ),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(100),
                commandVault.feedAllHingesFingersAUTO(),

                //
                commandVault.startIntakeProc(),
                new ParallelRaceGroup(
                    new FollowerCommand(follower, paths.ShootToHP, 1),
                    new WaitCommand(safeTime)
                ),
                new InstantCommand(follower::resumePathFollowing),
                new WaitCommand(wait_HP),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.HPToShoot, 1),
                        new SequentialCommandGroup(
                                commandVault.reverseIntake(),
                                new WaitCommand(500),
                                commandVault.stopIntakeProc()
                        )
                ),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(100),
                commandVault.feedAllHingesFingersAUTO(),

                /////////////////////////////////
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                commandVault.startIntakeProc(),
                                new ParallelRaceGroup(
                                        new FollowerCommand(follower, paths.ShootToHP, 1),
                                        new WaitCommand(safeTime)
                                ),
                                new InstantCommand(follower::resumePathFollowing),
                                new WaitCommand(wait_HP),
                                new ParallelCommandGroup(
                                        new FollowerCommand(follower, paths.HPToShoot, 1),
                                        new SequentialCommandGroup(
                                                commandVault.reverseIntake(),
                                                new WaitCommand(500),
                                                commandVault.stopIntakeProc()
                                        )
                                ),
                                commandVault.autonomousWaitForTurret(),
                                new WaitCommand(100),
                                commandVault.feedAllHingesFingersAUTO()
                        ),
                        new InstantCommand(),
                        () -> elapsedTime.getElapsedTimeSeconds() < 25.5
                ),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.ShootToPark),
                        commandVault.parkShooter()
                )
        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        follower.update();

        for (LynxModule hub : robotMap.getHubs()) hub.clearBulkCache();

        telemetry.addData("Loop Hz: ", 1.0/loopTime.getElapsedTimeSeconds());
        loopTime.resetTimer();

        FtcDashboard.getInstance().getTelemetry().addData("X", getPoseFTCCoor().getX());
        FtcDashboard.getInstance().getTelemetry().addData("Y", getPoseFTCCoor().getY());
        FtcDashboard.getInstance().getTelemetry().addData("Heading", getPoseFTCCoor().getTheta());
        FtcDashboard.getInstance().getTelemetry().update();
        telemetry.update();
    }


    public static class Paths {
        public PathChain StartToStack, StackToShoot, ShootToHP, HPToShoot, ShootToPark;

        public Paths(Follower follower) {
            double intake_x = 132.2;
            double wall_y = 10;
            StartToStack = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.3, 8.95),
                                    new Pose(84, 38),
                                    new Pose(135.0, 36.0)
                            ))
                    .setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(0, 0.07, HeadingInterpolator.constant(Math.toRadians(90))),
                            new HeadingInterpolator.PiecewiseNode(0.07, 0.5, HeadingInterpolator.tangent),
                            new HeadingInterpolator.PiecewiseNode(0.5, 1.0, HeadingInterpolator.constant(Math.toRadians(0)))
                    ))
                    .setBrakingStrength(0.7)
                    .build();

            StackToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134,36.0),
                                    new Pose(94.0, wall_y)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setBrakingStrength(4)
                    .build();

//            ShootToHP = follower.pathBuilder().addPath(
//                        new BezierLine(
//                                new Pose(94.0, wall_y),
//                                new Pose(intake_x, wall_y)
//                        )
//                ).setBrakingStrength(4)
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(350))
//                .addPath(
//                        new BezierLine(
//                                new Pose(intake_x, wall_y),
//                                new Pose(intake_x-2, wall_y)
//                        )
//                ).setBrakingStrength(4)
//                .setConstantHeadingInterpolation(Math.toRadians(350))
//                .addPath(
//                        new BezierLine(
//                                new Pose(intake_x, wall_y),
//                                new Pose(intake_x, wall_y)
//                        )
//                ).setBrakingStrength(4)
//                .setConstantHeadingInterpolation(Math.toRadians(350))
//                .build();

            ShootToHP = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(94.0, wall_y),
                                    new Pose(intake_x, wall_y)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(350))
                    .build();

            HPToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(intake_x, wall_y),
                                    new Pose(94.0, wall_y)
                            )
                    )
                    .setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(0, 0.2, HeadingInterpolator.linear(Math.toRadians(350), Math.toRadians(0))),
                            new HeadingInterpolator.PiecewiseNode(0.2, 1.0, HeadingInterpolator.constant(Math.toRadians(0)))
                    ))
                    .build();

            ShootToPark = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(94.0, wall_y),
                                    new Pose(104.0, 13)
                            )
                    ).setConstantHeadingInterpolation(0)
                    .build();
        }
    }

    public org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose getPoseFTCCoor() {
        Pose pedroPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()).getAsCoordinateSystem(FTCCoordinates.INSTANCE);

        return new org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose(pedroPose.getX(), pedroPose.getY(), Math.toDegrees(pedroPose.getHeading()));
    }

    @Override
    public void reset() {
        super.reset();
        PoseStorage.currentPose = getPoseFTCCoor();
    }
}