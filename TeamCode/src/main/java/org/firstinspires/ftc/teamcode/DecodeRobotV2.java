package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.GamepadExEx;
import org.firstinspires.ftc.teamcode.Hardware.PinpointYawWrapper;
import org.firstinspires.ftc.teamcode.Mechanisms.CommandSeriesVault;
import org.firstinspires.ftc.teamcode.Mechanisms.Detection;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Passthough;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Vector;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.Localization.PinpointLocalizer;

@Config
public class DecodeRobotV2 {
    public enum Alliance {
        RED,
        BLUE
    }
    protected Alliance alliance;

    protected FtcDashboard dashboard;
    protected GamepadExEx driverOp, toolOp;
    protected Telemetry telemetry;

    protected MecanumDrive drive = null;
    protected PinpointYawWrapper yawWrapper;

    private boolean hasInit = false;

    protected PinpointLocalizer teleOpLocalizer;

    // Mechanisms
    protected Intake intake;
    protected Passthough passthough;
    protected Shooter shooter;
    protected Detection detection;
    protected CommandSeriesVault commandSeriesVault;

    protected MotifStorage.Motif motif;

    public DecodeRobotV2(RobotMap robotMap, DriveConstants driveConstants, Alliance alliance,
                         Pose pose, MotifStorage.Motif motif
    ) {
        this.alliance = alliance;
        this.motif = motif;

        initCommon(robotMap, driveConstants);
        initTele(robotMap, pose);
//        this.initMechanismsTeleOp(robotMap);

        // Init Mechanisms when driver starts moving the robot
        new Trigger(() -> (Math.abs(drivetrainForward()) > 0.1 ||
            Math.abs(drivetrainStrafe()) > 0.1 ||
            Math.abs(drivetrainTurn()) > 0.1) && !hasInit)
            .whenActive(new InstantCommand(() -> this.initMechanismsTeleOp(robotMap)));
    }

    public DecodeRobotV2(RobotMap robotMap, DriveConstants driveConstants, Alliance alliance
    ) {
        this.alliance = alliance;

        initCommon(robotMap, driveConstants);
        initAuto();
    }

    /*-- Drive Commands --*/
    public double drivetrainStrafe() {
        return driverOp.getLeftX();
    }

    public double drivetrainForward() {
        return driverOp.getLeftY();
    }

    public double drivetrainTurn() {
        return driverOp.getRightX();
    }

    public void drive_update() {
        teleOpLocalizer.update();

        telemetry.addData("Pose", "X: %.2f, Y: %.2f, Theta: %.2f",
            getPose().getX(), getPose().getY(), getPose().getTheta());
        telemetry.addData("Alliance: ", getAlliance());
        telemetry.addData("MOTIF: ", getMotif());

        drive.drive(
            drivetrainStrafe(),
            drivetrainForward(),
            drivetrainTurn(),
            getHeading(),
            driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        );
    }

    public void drive_update(Pose pose) {
        drive.drive(
            pose.getX(),
            pose.getY(),
            pose.getTheta(),
            0,
            0
        );
    }

    /*-- Drive Type Pick --*/
    public void setFieldCentric() {
        drive.setFieldCentric();
    }

    public void setRobotCentric() {
        drive.setRobotCentric();
    }

    public void setAutoEnabled(boolean enabled) {
        drive.setAutoEnabled(enabled);
    }

    /*-- Getters --*/
    public double getHeading() {
        return yawWrapper.getRawYaw();
    }

    public double getContinuousHeading() {
        return yawWrapper.getContinuousYaw();
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public MotifStorage.Motif getMotif() {
        return motif;
    }

    public Pose getPose() {
        return teleOpLocalizer.getPose();
    }
    public Pose getPoseVelocity() {
        return teleOpLocalizer.getVelocity();
    }

//    public Pose getPose() {
//        return new Pose(0, 0, 0);
//    } // TODO
//    public Pose getPoseVelocity() {
//        return new Pose(0, 0, 0);
//    } // TODO

    /*-- Initializations --*/
    public void initCommon(RobotMap robotMap, DriveConstants driveConstants) {
        //- Camera
        this.dashboard = FtcDashboard.getInstance();

        //- Telemetries
        this.telemetry = robotMap.getTelemetry();

        //- Drive
        drive = new MecanumDrive(robotMap, driveConstants);
    }

    public void initAuto() {
        //- Setup and Initialize Mechanisms Objects
        initMechanismsAutonomous();
    }

    public void initTele(RobotMap robotMap, Pose startingPose) {
        teleOpLocalizer = new PinpointLocalizer(robotMap, startingPose);

        yawWrapper = new PinpointYawWrapper(
            robotMap,
            () -> (MathFunction.wrapDegrees(getPose().getTheta()) - (getAlliance() == Alliance.RED ? -90 : 90))
        );
        CommandScheduler.getInstance().registerSubsystem(yawWrapper);

        //- Gamepads
        this.driverOp = robotMap.getDriverOp();
        this.toolOp = robotMap.getToolOp();
    }

    /*-- Mechanisms Initialization --*/
    public void initMechanismsAutonomous() {
        //TODO: make init Mechanisms
    }

    public void initMechanismsTeleOp(RobotMap robotMap) {
        hasInit = true;

        driverOp.getGamepadButton(GamepadKeys.Button.START).whenPressed(yawWrapper::resetYawValue);

        intake = new Intake(robotMap);
        passthough = new Passthough(robotMap, getMotif());
        shooter = new Shooter(
            robotMap,
            this::getPose,
            alliance,
            true
        );
//        detection = new Detection(robotMap);

        commandSeriesVault = new CommandSeriesVault(intake, passthough, shooter);

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ConditionalCommand(
                commandSeriesVault.startIntakeProc(),
                commandSeriesVault.stopIntakeProc(),
                () -> intake.getState() != Intake.IntakeState.INTAKE
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ConditionalCommand(
                commandSeriesVault.feedOneFinger(0),
                new InstantCommand(),
                () -> shooter.turretInRange() && shooter.inLUTRange() && shooter.areWheelsEnabled()
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ConditionalCommand(
                commandSeriesVault.feedOneFinger(1),
                new InstantCommand(),
                () -> shooter.turretInRange() && shooter.inLUTRange() && shooter.areWheelsEnabled()
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ConditionalCommand(
                commandSeriesVault.feedOneFinger(2),
                new InstantCommand(),
                () -> shooter.turretInRange() && shooter.inLUTRange() && shooter.areWheelsEnabled()
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(new ConditionalCommand(
                commandSeriesVault.enableWheels(),
                commandSeriesVault.disableWheels(),
                () -> !shooter.areWheelsEnabled()
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                commandSeriesVault.flickFrontFinger()
        );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                commandSeriesVault.flickRearFinger()
        );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand(shooter::decrease_turret_offset)
        );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(shooter::increase_turret_offset)
        );

        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ConditionalCommand(
                commandSeriesVault.feedAllFingers(),
                new InstantCommand(),
                () -> shooter.turretInRange() && shooter.inLUTRange() && shooter.areWheelsEnabled()
        ));

        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.75).whenActive(new ConditionalCommand(
                commandSeriesVault.feedAllFingersMOTIF(),
                new InstantCommand(),
//                () -> true
                () -> shooter.turretInRange() && shooter.inLUTRange() && shooter.areWheelsEnabled()// && passthough.getShooting_order(0) != -1
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new InstantCommand(shooter::zeroTurret)
        );

        toolOp.getGamepadButton((GamepadKeys.Button.RIGHT_STICK_BUTTON)).whenPressed(
                commandSeriesVault.reverseIntake()
        );

        toolOp.getGamepadButton((GamepadKeys.Button.RIGHT_STICK_BUTTON)).whenReleased(
                commandSeriesVault.stopIntake()
        );

        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new InstantCommand(shooter::resetOffset)
        );

        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.75).whenActive(new ConditionalCommand(
                commandSeriesVault.parkShooter(),
                commandSeriesVault.unparkShooter(),
                () -> !shooter.isParked()
        ));

//        toolOp.getGamepadButton(GamepadKeys.Button.).whenPressed(
//                new InstantCommand(detection::setGoalPip)
//        );

        driverOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed( //11+1/8, 9+1/8
                new ConditionalCommand(
                        new InstantCommand(() -> teleOpLocalizer.setVector(
                                new Vector(72-(9+1.0/8.0), -24-(11+1.0/8.0)))
                        ),
                        new InstantCommand(() -> teleOpLocalizer.setVector(
                                new Vector(72-(9+1.0/8.0), 24+(11+1.0/8.0)))
                        ),
                        () -> getAlliance() != Alliance.BLUE
                )
        );

        toolOp.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(this::switchMotif);
    }

    public void switchMotif() {
        int idx = getMotif().ordinal() + 1;
        idx = idx > 2 ? 0 : idx;
        motif = MotifStorage.Motif.values()[idx];
        passthough.updateMotif(motif);
    }
}
