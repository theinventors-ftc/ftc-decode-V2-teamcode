package org.firstinspires.ftc.teamcode.AutoOPs;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterLimelight;
import org.firstinspires.ftc.teamcode.MotifStorage;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FollowerCommand;

import java.util.ArrayList;

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
        shooter = new Shooter(robotMap, this::getPoseFTCCoor, () -> new org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose(0, 0, 0), DecodeRobotV2.Alliance.RED, false);
        detection = new Detection(robotMap);
        commandVault = new CommandSeriesVault(intake, passthough, shooter, detection);

        commandVault.enableWheels().schedule();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        loopTime = new Timer();

        new SequentialCommandGroup(
                commandVault.autonomousWaitForTurret(),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.StartToHP1),
                new WaitCommand(500),
                commandVault.stopIntakeProc(),
                new FollowerCommand(follower, paths.HP1ToShoot),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(100),
                commandVault.feedAllFingers(),

                // Repeated Part

                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.ShootToHP),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.HPToShoot),
                        new SequentialCommandGroup(
                                commandVault.reverseIntake(),
                                new WaitCommand(500),
                                commandVault.stopIntakeProc()
                        )
                ),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(100),
                commandVault.feedAllFingers(),

                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.ShootToHP),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.HPToShoot),
                        new SequentialCommandGroup(
                                commandVault.reverseIntake(),
                                new WaitCommand(500),
                                commandVault.stopIntakeProc()
                        )
                ),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(100),
                commandVault.feedAllFingers(),

                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.ShootToHP),
                new WaitCommand(500),
                new ParallelCommandGroup(
                        new FollowerCommand(follower, paths.HPToShoot),
                        new SequentialCommandGroup(
                                commandVault.reverseIntake(),
                                new WaitCommand(500),
                                commandVault.stopIntakeProc()
                        )
                ),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(100),
                commandVault.feedAllFingers(),

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
//        ArrayList<Double> dists = shooter.getCachedDistances();
//        for (int i = 0; i < dists.size(); i++) {
//            telemetry.addData("Dist " + i, dists.get(i));
//        }
//        telemetry.addData("Dists", shooter.getCachedDistances());
        telemetry.update();
    }


    public static class Paths {
        public PathChain StartToHP1, HP1ToShoot, ShootToHP, HPToShoot, ShootToPark;

        public Paths(Follower follower) {
            StartToHP1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(101.9, 8.5),
                                    new Pose(134.0, 10.0)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(355))
                    .setBrakingStrength(2)
                    .build();

            HP1ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.0, 10.0),
                                    new Pose(96.0, 11.0)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(355), Math.toRadians(0))
                    .setBrakingStrength(2)
                    .build();

            ShootToHP = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.0, 11.0),
                                    new Pose(132.5, 14.0)
                            )
                    ).setConstantHeadingInterpolation(0)
                    .setBrakingStrength(2)
                    .build();

            HPToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.5, 14.0),
                                    new Pose(96.0, 11.0)
                            )
                    ).setConstantHeadingInterpolation(0)
                    .setBrakingStrength(2)
                    .build();

            ShootToPark = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.0, 11.0),
                                    new Pose(104.0, 12.0)
                            )
                    ).setConstantHeadingInterpolation(0)
                    .setBrakingStrength(2)
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