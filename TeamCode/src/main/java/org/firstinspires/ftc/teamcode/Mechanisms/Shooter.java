package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Controllers.PIDFEx;
import org.firstinspires.ftc.teamcode.Controllers.PIDFExCoeffs;
import org.firstinspires.ftc.teamcode.Controllers.StateMachine;
import org.firstinspires.ftc.teamcode.DecodeRobotV2;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

@Config
public class Shooter extends SubsystemBase {
    // ---------------------------------------- Hardware ---------------------------------------- //
    private MotorExEx wheel1, wheel2;
    private ServoImplEx hoodServo;
    private MotorExEx turretMotor;

    // ---------------------------------------- Constants --------------------------------------- //
    // Wheel
    private static final int WHEEL_TICKS_PER_REV = 28, WHEEL_MAX_RPM = 5800;
    private static final double MAX_TICKS_PER_S = 2700; // WHEEL_MAX_RPM/60.0 * 28

    // Hood
    private static final double MIN_HOOD_POS = 0.91, MAX_HOOD_POS = 0.05;

    // Turret
    private static final double TICKS_PER_FULL_ROTATION = 1916.0;
    private static final double MAX_TURRET_POWER = 1.0;
    private static final double MIN_TURRET_ANGLE = -65.0, MAX_TURRET_ANGLE = 192.0;

    // ----------------------------------------- States ----------------------------------------- //
    private boolean wheelsEnabled = false;
    private boolean turretLockEnabled = false;
    private boolean hoodLockEnabled = false;

    // ---------------------------------------- Poses ------------------------------------------- //
    private Supplier<Pose> curPose;
    private final Pose REDGoalPose = new Pose(69.0, -67.0, 0);
    private final Pose BLUEGoalPose = new Pose(69.0, 67.0, 0);
    private final Pose goalPose;

    // ---------------------------------- Controllers and LUTs ---------------------------------- //
    private InterpLUT wheelSpeed, hoodAngle;
    private PIDFEx turretController, veloController;
    private PIDFExCoeffs coeffsTurret, coeffsVelo;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 1, 0);

    // ------------------------------------ Turret Zeroing -------------------------------------- //
    private boolean turretZeroed = false;
    private double turretZeroPower = -0.25;
    private double turretZeroCurrentThreshold = 2.1;
    public static double turretZeroOffset = 102.5;
    private StateMachine hasStalled;

    // ------------------------------------------ Util ------------------------------------------ //
    private Telemetry telemetry;
    private DoubleSupplier voltage;
    private ArrayList<Double> cachedDistances = new ArrayList<>();

    public static double vel = 0.0, hoodVal = 0.0;

    public Shooter(RobotMap robotMap, Supplier<Pose> curPose, DecodeRobotV2.Alliance alliance, boolean doZero) {
        this.wheel1 = robotMap.getShooterWheel1Motor();
        this.wheel2 = robotMap.getShooterWheel2Motor();
        this.hoodServo = robotMap.getHoodServo();
        this.turretMotor = robotMap.getTurretMotor();
        turretMotor.setInverted(true);
        turretMotor.resetEncoder();
        turretZeroed = !doZero;
        turretZeroed = true;
        this.telemetry = robotMap.getTelemetry();

        hasStalled = new StateMachine(() -> ((DcMotorEx)turretMotor.getRawMotor()).getCurrent(CurrentUnit.AMPS) > turretZeroCurrentThreshold, 400);

        wheel1.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        wheel1.setInverted(true);
        wheel2.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        wheel2.setInverted(true);

        // Select Correct Goal Based On Alliance
        goalPose = (alliance == DecodeRobotV2.Alliance.RED) ? REDGoalPose : BLUEGoalPose;

        coeffsTurret = new PIDFExCoeffs(
                0.07,
                0.16,
                0.0018,
                0.0,
                0.1,
                0.01,
                16,
                0.5
        );
        turretController = new PIDFEx(coeffsTurret);

        coeffsVelo = new PIDFExCoeffs(
                13.5,
                0.0,
                0.0,
                0.0,
                0.0,
                3,
                600,
                0.8
        );
        veloController = new PIDFEx(coeffsVelo);

        this.curPose = curPose;

        // Initialize LUTs here
        wheelSpeed = new InterpLUT();
        hoodAngle = new InterpLUT();

        wheelSpeed.add(39.23, 0.61);
        wheelSpeed.add(51.89, 0.63);
        wheelSpeed.add(71.0, 0.67);
        wheelSpeed.add(86.29, 0.725);
        wheelSpeed.add(101.22, 0.76);
        wheelSpeed.add(115.59, 0.812);
        wheelSpeed.add(133.36, 0.869);
        wheelSpeed.add(147.45, 0.9);
        wheelSpeed.add(165.22, 0.965);

        hoodAngle.add(39.23, 0.24);
        hoodAngle.add(51.89, 0.3);
        hoodAngle.add(71.0, 0.67);
        hoodAngle.add(86.29, 0.84);
        hoodAngle.add(101.22, 0.9);
        hoodAngle.add(115.59, 0.95);
        hoodAngle.add(133.36, 0.98);
        hoodAngle.add(147.45, 1.0);
        hoodAngle.add(165.22, 1.0);

        wheelSpeed.createLUT();
        hoodAngle.createLUT();

        voltage = () -> robotMap.getBattery().getVoltage();
    }

    @Override
    public void periodic() {
        if(!turretZeroed) {
            hasStalled.update();
            turretMotor.set(turretZeroPower);
            if(hasStalled.isJustActive()) {
                turretMotor.resetEncoder();
                turretMotor.set(0);
                turretZeroed = true;
            }
            return;
        }

        // ------------------------------------- Telemetry -------------------------------------- //
        telemetry.addData("[Shooter] Wheel State ", wheelsEnabled);
        telemetry.addData("[Shooter] Turret Lock ", turretLockEnabled);
        telemetry.addData("[Shooter] Hood Lock ", hoodLockEnabled);
        telemetry.addData("[Shooter] Turret Ticks: ", turretMotor.getCurrentPosition());
        telemetry.addData("[Shooter] Turret Angle: ", getTurretAngle());
        telemetry.addData("[Shooter] Goal Dist: ", getDistanceToGoal());
        telemetry.addData("[Shooter] Goal Angle: ", getAngleToGoal());

        // --------------------------------------- Turret --------------------------------------- //
        turretController.setSetPoint(Range.clip(getAngleToGoal(), MIN_TURRET_ANGLE, MAX_TURRET_ANGLE));

        turretMotor.set(Range.clip(
                turretController.calculate(getTurretAngle()),
                -MAX_TURRET_POWER,
                MAX_TURRET_POWER
        ));

        if(!inLUTRange()) return;

        // ---------------------------------------- Hood ---------------------------------------- //
        hoodServo.setPosition(Range.scale(
                hoodAngle.get(getDistanceToGoal()), 0, 1, MIN_HOOD_POS, MAX_HOOD_POS)
        );

        // --------------------------------------- Wheels --------------------------------------- //
        if(wheelsEnabled) {
            wheel1.set(getControlledWheelPower(wheelSpeed.get(getDistanceToGoal())*0.965));
            wheel2.set(getControlledWheelPower(wheelSpeed.get(getDistanceToGoal())*0.965));
        }
    }

    public void cacheCurrentDistance() {
        double dist = getDistanceToGoal();
        cachedDistances.add(dist);
    }

    public ArrayList<Double> getCachedDistances() {
        return cachedDistances;
    }

    // ----------------------------------------- Wheels ----------------------------------------- //
    public double getControlledWheelPower(double power) {
        double speed = 0.9 * power * MAX_TICKS_PER_S;
        veloController.setSetPoint(speed);
        double velocity = veloController.calculate(-wheel1.getCorrectedVelocity()) +
                feedforward.calculate(speed, -wheel1.getAcceleration());

        FtcDashboard.getInstance().getTelemetry().addData("[Shooter] Velo Target: ", speed);
        FtcDashboard.getInstance().getTelemetry().addData("[Shooter] Velo Actual: ", -wheel1.getCorrectedVelocity());
        return velocity / MAX_TICKS_PER_S;
    }

    public void enableWheels() {
        wheelsEnabled = true;
    }

    public void disableWheels() {
        wheelsEnabled = false;
        wheel1.set(0);
        wheel2.set(0);
    }

    public boolean areWheelsEnabled() {
        return wheelsEnabled;
    }

    public boolean wheelsAtSpeed() {
        return Math.abs(veloController.getPositionError()) < 50;
    }

    // ----------------------------------------- Turret ----------------------------------------- //
    public double getTurretAngle() {
        return (((turretMotor.getCurrentPosition())%TICKS_PER_FULL_ROTATION)*360.0/TICKS_PER_FULL_ROTATION)*(180.0/181.4) - turretZeroOffset;
    }

    public boolean turretInRange() {
        double angleToGoal = getAngleToGoal();
        return angleToGoal > -90 && angleToGoal < 225;
    }

    public boolean turretAtGoal() {
        return Math.abs(turretController.getPositionError()) < 1.2;
    }

    // ---------------------------------------- IK Stuff ---------------------------------------- //
    public double getDistanceToGoal() {
        Pose pose = curPose.get();
        double dx = goalPose.getX() - pose.getX();
        double dy = goalPose.getY() - pose.getY();
        return Math.hypot(dx, dy);
    }

    public double getAngleToGoal() {
        double dx = goalPose.getX() - curPose.get().getX();
        double dy = goalPose.getY() - curPose.get().getY();

        double targetAngle = Math.toDegrees(Math.atan2(dy, dx));

        double robotHeading = curPose.get().getTheta() % 360;
        if (robotHeading >= 180) robotHeading -= 360;
        if (robotHeading < -180) robotHeading += 360;

        double relativeAngle = targetAngle - robotHeading;
        relativeAngle %= 360;
        if (relativeAngle >= 180) relativeAngle -= 360;
        if (relativeAngle < -180) relativeAngle += 360;

        if (relativeAngle < -95) relativeAngle = -95;
        if (relativeAngle > 205) relativeAngle = 205;

        return relativeAngle;
    }

    public boolean inLUTRange() {
        double dist = getDistanceToGoal();
        return dist > 39.24 && dist < 165.21;
    }

    public void zeroTurret() {
        turretZeroed = false;
    }
}
