package org.firstinspires.ftc.teamcode.AutoOPs.Houston;

import com.acmerobotics.dashboard.FtcDashboard;
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

@Autonomous(name = "RED_FAR", group = "Autonomous")
@Configurable
public class RED_FAR extends CommandOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private RobotMap robotMap;

    private Detection detection;

    private Intake intake;
    private Passthough passthough;
    private Shooter shooter;

    private CommandSeriesVault commandVault;

    private Timer loopTime;

    private Paths paths;
    private long safeTime = 2500;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset(); // Ultra SOS
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        robotMap = new RobotMap(hardwareMap, telemetry,null,null);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(101.9, 8.5, Math.toRadians(0)));
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
                true,
                true
        );
        detection = new Detection(robotMap);
        commandVault = new CommandSeriesVault(intake, passthough, shooter, detection);

        commandVault.enableWheels().schedule();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        loopTime = new Timer();

        new SequentialCommandGroup(
                commandVault.autonomousWaitForTurret(),
                commandVault.feedAllHingesFingersAUTO(),
                commandVault.startIntakeProc(),
                new ParallelRaceGroup(
                    new FollowerCommand(follower, paths.StartToHP1, 1),
                    new WaitCommand(safeTime)
                ),
                new InstantCommand(follower::resumePathFollowing),
                commandVault.stopIntakeProc(),
                new FollowerCommand(follower, paths.HP1ToShoot,1),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(100),
                commandVault.feedAllHingesFingersAUTO(),

                // Repeated Part

                commandVault.startIntakeProc(),
                new ParallelRaceGroup(
                    new FollowerCommand(follower, paths.ShootToHP, 1),
                    new WaitCommand(safeTime)
                ),
                new InstantCommand(follower::resumePathFollowing),
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

                commandVault.startIntakeProc(),
                new ParallelRaceGroup(
                    new FollowerCommand(follower, paths.ShootToHP, 1),
                    new WaitCommand(safeTime)
                ),
                new InstantCommand(follower::resumePathFollowing),
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

                commandVault.startIntakeProc(),
                new ParallelRaceGroup(
                        new FollowerCommand(follower, paths.ShootToHP, 1),
                        new WaitCommand(safeTime)
                ),
                new InstantCommand(follower::resumePathFollowing),
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

                commandVault.startIntakeProc(),
                new ParallelRaceGroup(
                    new FollowerCommand(follower, paths.ShootToHP, 1),
                    new WaitCommand(safeTime)
                ),
                new InstantCommand(follower::resumePathFollowing),
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

                //
                new FollowerCommand(follower, paths.ShootToPark),
                commandVault.parkShooter()
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
        public PathChain StartToHP1, HP1ToShoot, ShootToHP, HPToShoot, ShootToPark;

        public Paths(Follower follower) {
            double wall_x = 122.5, wall_x_final = 131;
            double wall_y = 24, wall_y_final = 11;
            StartToHP1 = follower.pathBuilder().addPath(
                    new BezierLine(
                            new Pose(101.9, 8.5),
                            new Pose(wall_x, wall_y)
                    ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(330))
                .addPath(
                        new BezierLine(
                            new Pose(wall_x, wall_y),
                            new Pose(wall_x_final, wall_y_final)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(330), Math.toRadians(340))
                .build();

            HP1ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(wall_x_final, wall_y_final),
                                    new Pose(94.0, 14.05)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(340), Math.toRadians(0))
                    .build();

            ShootToHP = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(94.0, 14.05),
                                    new Pose(wall_x, wall_y)
                            ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(330))
                .addPath(
                        new BezierLine(
                            new Pose(wall_x, wall_y),
                            new Pose(wall_x_final, wall_y_final)
                        )
                    ).setLinearHeadingInterpolation(Math.toRadians(330), Math.toRadians(340))
                    .build();

            HPToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(wall_x_final, wall_y_final),
                                    new Pose(124, 26.5),
                                    new Pose(94.0, 14.05)
                            )
                    )
                    .setHeadingInterpolation(HeadingInterpolator.piecewise(
                            new HeadingInterpolator.PiecewiseNode(0, 0.8, HeadingInterpolator.linear(Math.toRadians(340), Math.toRadians(0))),
                            new HeadingInterpolator.PiecewiseNode(0.8, 1.0, HeadingInterpolator.constant(Math.toRadians(0)))
                    ))
                    .build();

            ShootToPark = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(94.0, 14.05),
                                    new Pose(104.0, 14.05)
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