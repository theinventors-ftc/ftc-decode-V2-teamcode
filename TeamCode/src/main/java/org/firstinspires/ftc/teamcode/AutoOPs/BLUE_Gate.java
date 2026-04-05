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
import org.firstinspires.ftc.teamcode.Mechanisms.Detection;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Passthough;
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterLimelight;
import org.firstinspires.ftc.teamcode.MotifStorage;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FollowerCommand;

import java.util.ArrayList;

@Autonomous(name = "BLUE_Gate", group = "Autonomous")
@Configurable
public class BLUE_Gate extends CommandOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private RobotMap robotMap;

    private Detection detection;

    private Intake intake;
    private Passthough passthough;
    private ShooterLimelight shooter;

    private CommandSeriesVault commandVault;

    private Timer loopTime;

    private Paths paths;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset(); // Ultra SOS
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        robotMap = new RobotMap(hardwareMap, telemetry,null,null);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144-(120 - 5.625), 128.89763779, Math.toRadians(180)));
        paths = new Paths(follower);

        intake = new Intake(robotMap);
        passthough = new Passthough(robotMap, MotifStorage.Motif.PPG);
        shooter = new ShooterLimelight(robotMap, this::getPoseFTCCoor, DecodeRobotV2.Alliance.BLUE, false);
        detection = new Detection(robotMap);
        commandVault = new CommandSeriesVault(intake, passthough, shooter, detection);

        commandVault.enableWheels().schedule();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        loopTime = new Timer();

        new SequentialCommandGroup(
                new FollowerCommand(follower, paths.StartToGoal),
                commandVault.autonomousWaitForTurret(),
//                new InstantCommand(shooter::cacheCurrentDistance),
                commandVault.feedAllFingers(),
                new WaitCommand(100),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.GoalToIntakeStack2, 0.9, true),
                new InstantCommand(follower::resumePathFollowing),
                new WaitCommand(550),
                new FollowerCommand(follower, paths.IntakeStack2ToLaunchArea2, 1),
                commandVault.stopIntakeProc(),
                new WaitCommand(160),
                commandVault.autonomousWaitForTurret(),
                new WaitCommand(150),
                commandVault.feedAllFingers(),
                commandVault.startIntakeProc(),
                new FollowerCommand(follower, paths.LauchArea2ToGate),
                new WaitCommand(1550),
                new FollowerCommand(follower, paths.GateToLaunchArea2, 1),
                commandVault.stopIntakeProc(),
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
                new FollowerCommand(follower, paths.LaunchArea1ToParking, 1, true)
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
                IntakeStack2ToLaunchArea2,
                LauchArea2ToGate,
                GateToLaunchArea2,
                LauchArea2ToIntakeStack1,
                Intake1ToLauchArea1,
                LaunchArea1ToParking;

        public Paths(Follower follower) {
            StartToGoal = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144 - 108.0, 135.0),
                                    new Pose(144 - 66, 80)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(160))
                    .build();

            GoalToIntakeStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144 - 66, 80),
                                    new Pose(144 - 75.5, 70),
                                    new Pose(144 - 93.0, 52.0),
                                    new Pose(144 - 136.8, 60)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .setBrakingStrength(1.3)
                    .build();

            IntakeStack2ToLaunchArea2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(144 - 136.8, 60),
                                    new Pose(144 - 95, 68),
                                    new Pose(144 - 87.5, 83.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            LauchArea2ToGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144 - 87.5, 83.5),
                                    new Pose(144 - 125.5, 65)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(150))
                    .setBrakingStrength(1.9)
                    .build();

            GateToLaunchArea2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144 - 125.5, 65),
                                    new Pose(144 - 84, 83.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(130))
                    .build();

            LauchArea2ToIntakeStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144-84, 83.5),
                                    new Pose(144-129.5, 83.5)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .setBrakingStrength(2.4)
                    .build();

            Intake1ToLauchArea1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144-129.5, 83.5),
                                    new Pose(144-86.0, 83.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(210))
                    .build();

            LaunchArea1ToParking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(144 - 86, 83.5),
                                    new Pose(144 - 87.5, 60)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
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