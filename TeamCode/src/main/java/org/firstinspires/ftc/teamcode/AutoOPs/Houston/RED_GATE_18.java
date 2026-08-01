package org.firstinspires.ftc.teamcode.AutoOPs.Houston;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
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

@Autonomous(name = "RED_GATE_18", group = "Autonomous")
@Configurable
public class RED_GATE_18 extends CommandOpMode {
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
                              this::getAccelPoseFTCCoor, DecodeRobotV2.Alliance.RED,
                              false, true, true, true);
        commandVault = new CommandSeriesVault(intake, passthough, shooter);

        commandVault.enableWheels().schedule();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        loopTime = new Timer();

        long gateWaitTime = 1500;
        double preloadShootAngle = 170, stack2ShootAngle = 53, stack1ShootAngle = 80;
        double[] gateShootAngle = {53, 52, 52};

        new SequentialCommandGroup(
            new InstantCommand(() -> shooter.enableAutoCustom(preloadShootAngle)),
            new ParallelCommandGroup(
                new FollowerCommand(follower, paths.StartToStack2, 0.5, false),
                new SequentialCommandGroup(
                    new WaitCommand(650),
                    new WaitUntilCommand(() -> shooter.getDistanceToGoal() > 30),
                    new InstantCommand(() -> follower.setMaxPower(0.2)),
                    commandVault.feedAllHingesFingersAUTO(),
                    new WaitCommand(100),
                    new InstantCommand(() -> follower.setMaxPower(1)),
                    commandVault.startIntakeProc(),
                    new WaitCommand(400),
                    new InstantCommand(() -> shooter.enableAutoCustom(stack2ShootAngle))
                )
            ),

            new InstantCommand(follower::resumePathFollowing),
            new WaitCommand(300),

            new ParallelRaceGroup(
                new ParallelCommandGroup(
                    new InstantCommand(() -> shooter.enableAutoCustom(stack2ShootAngle)),
                    new FollowerCommand(follower, paths.Stack2ToShoot,1, false, true),
                    new SequentialCommandGroup(
                        new WaitCommand(300),
                        commandVault.reverseIntake(),
                        new WaitCommand(500),
                        commandVault.stopIntakeProc()
                    ),
                    new WaitCommand(300000)
                ),
                new WaitUntilCommand(() -> follower.getPose().getX() <= 95)
            ),
            commandVault.feedAllHingesFingersAUTO(),
            commandVault.startIntakeProc(),

            // 1

            new FollowerCommand(follower, paths.ShootToGate,1, false, true),
            commandVault.waitAtGate(),

            new ParallelRaceGroup(
                new ParallelCommandGroup(
                    new InstantCommand(() -> shooter.enableAutoCustom(gateShootAngle[0])),
                    new FollowerCommand(follower, paths.GateToShoot,1, false, true),
                    new SequentialCommandGroup(
                        commandVault.reverseIntake(),
                        new WaitCommand(500),
                        commandVault.stopIntakeProc()
                    ),
                    new WaitCommand(300000)
                ),
                new WaitUntilCommand(() -> follower.getPose().getX() <= 95)
            ),
            commandVault.feedAllHingesFingersAUTO(),
            commandVault.startIntakeProc(),

            // 2

            new FollowerCommand(follower, paths.ShootToGate,1, false, true),
                commandVault.waitAtGate(),

            new ParallelRaceGroup(
                new ParallelCommandGroup(
                    new InstantCommand(() -> shooter.enableAutoCustom(gateShootAngle[1])),
                    new FollowerCommand(follower, paths.GateToShoot,1, false, true),
                    new SequentialCommandGroup(
                        commandVault.reverseIntake(),
                        new WaitCommand(500),
                        commandVault.stopIntakeProc()
                    ),
                    new WaitCommand(300000)
                ),
                new WaitUntilCommand(() -> follower.getPose().getX() <= 95)
            ),
            commandVault.feedAllHingesFingersAUTO(),
            commandVault.startIntakeProc(),

            // 3

            new FollowerCommand(follower, paths.ShootToGate,1, false, true),
            commandVault.waitAtGate(),

            new ParallelRaceGroup(
                new ParallelCommandGroup(
                    new InstantCommand(() -> shooter.enableAutoCustom(gateShootAngle[2])),
                    new FollowerCommand(follower, paths.GateToShoot,1, false, true),
                    new SequentialCommandGroup(
                        commandVault.reverseIntake(),
                        new WaitCommand(500),
                        commandVault.stopIntakeProc()
                    ),
                    new WaitCommand(300000)
                ),
                new WaitUntilCommand(() -> follower.getPose().getX() <= 95)
            ),
            commandVault.feedAllHingesFingersAUTO(),
            commandVault.startIntakeProc(),

            //////////////////////////////////////////

            new FollowerCommand(follower, paths.ShootToStack1,1, false),
            new InstantCommand(follower::resumePathFollowing),
            new WaitCommand(200),

            new ParallelRaceGroup(
                new ParallelCommandGroup(
                    new InstantCommand(() -> shooter.enableAutoCustom(stack1ShootAngle)),
                    new FollowerCommand(follower, paths.Stack1ToShoot,1, false, true),
                    new SequentialCommandGroup(
                        commandVault.reverseIntake(),
                        new WaitCommand(600),
                        commandVault.stopIntakeProc()
                    ),
                    new WaitCommand(3000000)
                ),
                new WaitUntilCommand(() -> follower.getPose().getX() <= 102)
            ),
            commandVault.stopIntakeProc(),
            commandVault.feedAllHingesFingersAUTO(),
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

        telemetry.addData("X", getPoseFTCCoor().getX());
        telemetry.addData("Y", getPoseFTCCoor().getY());
        telemetry.addData("Heading", getPoseFTCCoor().getTheta());
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
                Stack1ToShoot;

        public Paths(Follower follower) {
            StartToStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(117.5, 130.5),
                                    new Pose(68, 69),
                                    new Pose(83, 54.8),
                                    new Pose(94.4, 57.9),
                                    new Pose(133.5, 58)
                            )
                    ).setTangentHeadingInterpolation()
                    .setBrakingStrength(4)
                    .build();

            Stack2ToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(133.5, 60),
                                    new Pose(99, 58),
                                    new Pose(90, 75)
                            )
                    ).setConstantHeadingInterpolation(0)
//                    .setBrakingStart(1.2)
                    .setBrakingStrength(4)
                    .build();

            ShootToGate = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90, 75),
                                    new Pose(131.5, 62)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .setBrakingStart(2)
                    .setBrakingStrength(0.5)
                    .build();

            GateToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.5, 62),
                                    new Pose(90, 75)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .setBrakingStart(1.2)
                    .setBrakingStrength(0.5)
                    .build();

            ShootToStack1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90, 83.5),
                                    new Pose(128, 83.5)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))
                    .setBrakingStrength(4)
                    .build();

            Stack1ToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(127.5, 83.5),
                                    new Pose(87, 105.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(315))
                    .setBrakingStart(1.2)
                    .setBrakingStrength(0.5)
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