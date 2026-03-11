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
    private static final double MIN_TURRET_ANGLE = -90.0, MAX_TURRET_ANGLE = 188.0;

    // ----------------------------------------- States ----------------------------------------- //
    private boolean wheelsEnabled = false;
    private boolean turretLockEnabled = true;
    private boolean hoodLockEnabled = true;
    private boolean parking_state = false;

    // ---------------------------------------- Poses ------------------------------------------- //
    private Supplier<Pose> curPose;
    private final Pose REDGoalPose = new Pose(69.0, -67.5, 0);
    private final Pose BLUEGoalPose = new Pose(69.0, 67.5, 0);
    private final Pose goalPose;

    // ---------------------------------- Controllers and LUTs ---------------------------------- //
    private InterpLUT wheelSpeed, hoodAngle;
    private PIDFEx turretController, veloController;
    private PIDFExCoeffs coeffsTurret, coeffsVelo;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 1, 0);

    // ------------------------------------ Turret Zeroing -------------------------------------- //
    private boolean turretZeroed = false;
    private double turretZeroPower = -0.25;
    private double turretZeroCurrentThreshold = 2.0;
    public static double turretZeroOffset = 102;
    private StateMachine hasStalled;

    // ------------------------------------------ Util ------------------------------------------ //
    private Telemetry telemetry;
    private DoubleSupplier voltage;
    private ArrayList<Double> cachedDistances = new ArrayList<>();

    public Shooter(RobotMap robotMap, Supplier<Pose> curPose, DecodeRobotV2.Alliance alliance, boolean doZero) {
        this.wheel1 = robotMap.getShooterWheel1Motor();
        this.wheel2 = robotMap.getShooterWheel2Motor();
        this.hoodServo = robotMap.getHoodServo();
        this.turretMotor = robotMap.getTurretMotor();
        turretMotor.setInverted(true);
        turretMotor.resetEncoder();
        turretZeroed = !doZero;
        this.telemetry = robotMap.getTelemetry();

        hasStalled = new StateMachine(() -> ((DcMotorEx)turretMotor.getRawMotor()).getCurrent(CurrentUnit.AMPS) > turretZeroCurrentThreshold, 300);

        wheel1.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
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
                14.5,
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
        wheelSpeed.add(63.6, 0.655);
        wheelSpeed.add(71.0, 0.675);
        wheelSpeed.add(86.29, 0.725);
        wheelSpeed.add(101.22, 0.76);
        wheelSpeed.add(115.59, 0.81);
        wheelSpeed.add(128.0, 0.81/0.964); //
        wheelSpeed.add(143.52, 0.83/0.964); //
        wheelSpeed.add(151.14, 0.86/0.964); //
        wheelSpeed.add(165.22, 0.91/0.964);

        hoodAngle.add(39.23, 0.24);
        hoodAngle.add(51.89, 0.3);
        hoodAngle.add(63.6, 0.675);
        hoodAngle.add(65.0, 0.675);
        hoodAngle.add(71.0, 0.675);
        hoodAngle.add(86.29, 0.84);
        hoodAngle.add(101.22, 0.9);
        hoodAngle.add(115.59, 0.95);
        hoodAngle.add(127.0, 1.0);
        hoodAngle.add(133.36, 1.0); //
        hoodAngle.add(143.52, 1.0); //
        hoodAngle.add(151.14, 1.0); //
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

        if(parking_state) {
            turretController.setSetPoint(-101);
            turretMotor.set(Range.clip(
                    turretController.calculate(getTurretAngle()),
                    -MAX_TURRET_POWER,
                    MAX_TURRET_POWER
            ));

            hoodServo.setPosition(MIN_HOOD_POS);
            return;
        }

        // ------------------------------------- Telemetry -------------------------------------- //
        telemetry.addData("[Shooter] Wheel State ", wheelsEnabled);
        telemetry.addData("[Shooter] Turret Lock ", turretLockEnabled);
        telemetry.addData("[Shooter] Hood Lock ", hoodLockEnabled);
        telemetry.addData("[Shooter] Turret Angle: ", getTurretAngle());
        telemetry.addData("[Shooter] GOAL Dist: ", getDistanceToGoal());
        telemetry.addData("[Shooter] GOAL Angle: ", getAngleToGoal());

        // --------------------------------------- Turret --------------------------------------- //
        turretController.setSetPoint(Range.clip(
                turretLockEnabled ? getAngleToGoal() : 0,
                MIN_TURRET_ANGLE,
                MAX_TURRET_ANGLE)
        );

        turretMotor.set(Range.clip(
                turretController.calculate(getTurretAngle()),
                -MAX_TURRET_POWER,
                MAX_TURRET_POWER
        ));

        if(!inLUTRange()) return;

        // ---------------------------------------- Hood ---------------------------------------- //
        hoodServo.setPosition(Range.scale(
                (hoodLockEnabled ? hoodAngle.get(getDistanceToGoal()) : 0),
                0,
                1,
                MIN_HOOD_POS,
                MAX_HOOD_POS
        ));

        // --------------------------------------- Wheels --------------------------------------- //
        if(wheelsEnabled) {
            wheel1.set(getControlledWheelPower(wheelSpeed.get(getDistanceToGoal())*0.964));
            wheel2.set(getControlledWheelPower(wheelSpeed.get(getDistanceToGoal())*0.964));
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
        double velocity = veloController.calculate(wheel1.getCorrectedVelocity()) +
                feedforward.calculate(speed, wheel1.getAcceleration());
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
        return (((turretMotor.getCurrentPosition())%TICKS_PER_FULL_ROTATION)*360.0/TICKS_PER_FULL_ROTATION)*(180.0/181.4)*(178.0/180.0) - turretZeroOffset;
    }

    public boolean turretInRange() {
        double angleToGoal = getAngleToGoal();
        return angleToGoal > MIN_TURRET_ANGLE && angleToGoal < MAX_TURRET_ANGLE;
    }

    public boolean turretAtGoal() {
        return Math.abs(turretController.getPositionError()) < (atSmallTriangle() ? 1.2 : 0.7);
    }

    // ---------------------------------------- IK Stuff ---------------------------------------- //
    public double getDistanceToGoal() {
        Pose pose = curPose.get();
        double dx = goalPose.getX() - pose.getX();
        double dy = goalPose.getY() - pose.getY();
        return Math.hypot(dx, dy);
    }

    public double getAngleToGoal() {
        double dx_ref = goalPose.getX() - curPose.get().getX();
        double dy_ref = goalPose.getY() - curPose.get().getY();

        double targetAngle_ref = Math.toDegrees(Math.atan2(dy_ref, dx_ref));
        double dx = dx_ref;
        double dy = dy_ref;

        if(Math.abs(targetAngle_ref) > 70.0) dx += Range.scale(targetAngle_ref, -70.0, -90.0, -1.5, -0.8);
        if(atSmallTriangle()) dy += 0;

        double targetAngle = Math.toDegrees(Math.atan2(dy, dx));

        double robotHeading = curPose.get().getTheta() % 360;
        if (robotHeading >= 180) robotHeading -= 360;
        if (robotHeading < -180) robotHeading += 360;

        double relativeAngle = targetAngle - robotHeading;
        relativeAngle %= 360;
        if (relativeAngle >= 180) relativeAngle -= 360;
        if (relativeAngle < -180) relativeAngle += 360;

        relativeAngle = Range.clip(relativeAngle, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);

        return relativeAngle;
    }

    public boolean inLUTRange() {
        double dist = getDistanceToGoal();
        return dist > 39.24 && dist < 165.21;
    }

    public boolean atSmallTriangle() {
        return getDistanceToGoal() > (325.0/2.54);
    }

    public void zeroTurret() {
        turretZeroed = false;
    }

    public void enableParkingState() {
        parking_state = true;
    }

    public void disableParkingState() {
        parking_state = false;
    }

    public boolean isParked(){
        return parking_state;
    }

    public void increase_turret_offset() { turretZeroOffset += 1.0; }
    public void decrease_turret_offset() { turretZeroOffset -= 1.0; }

}
