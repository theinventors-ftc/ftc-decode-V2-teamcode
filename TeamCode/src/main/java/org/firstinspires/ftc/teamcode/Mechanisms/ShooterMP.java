package org.firstinspires.ftc.teamcode.Mechanisms;

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
import org.firstinspires.ftc.teamcode.Controllers.SigmoidPositionProfile;
import org.firstinspires.ftc.teamcode.DecodeRobotV2;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.function.DoubleSupplier;

@Config
public class ShooterMP extends SubsystemBase {
    // ---------------------------------------- Hardware ---------------------------------------- //
    private MotorExEx wheel1, wheel2;
    private ServoImplEx hoodServo;
//    private CRServoImplEx turretServoDriver, turretServoFollower;
    private MotorExEx turretMotor;
//    private AnalogInput turretServoPot;

    // ---------------------------------------- Constants --------------------------------------- //
    // Wheel
    private static final int WHEEL_TICKS_PER_REV = 28, WHEEL_MAX_RPM = 5800;
    private static final double MAX_TICKS_PER_S = 2700; // WHEEL_MAX_RPM/60.0 * 28

    // Hood
    private static final double MIN_HOOD_POS = 0.95, MAX_HOOD_POS = 0.07;

    // Turret
    private static final double TICKS_PER_FULL_ROTATION = 1916.0;
    private static final double TURRET_MULTIPLIER = 1.0, MAX_TURRET_POWER = 1.0;

    // --------------------------- Turret Angle Calculation Variables --------------------------- //
    private double turns = 0, prevRawTurretServoAngle = 0;

    // ----------------------------------------- States ----------------------------------------- //
    private boolean wheelsEnabled = false;
    private boolean turretLockEnabled = false;
    private boolean hoodLockEnabled = false;

    // ---------------------------------------- Poses ------------------------------------------- //
    private Supplier<Pose> curPose;
    private final Pose REDGoalPose = new Pose(69, -69, 0); // TODO: Recheck
    private final Pose BLUEGoalPose = new Pose(69, 69, 0); // TODO: Recheck
    private final Pose goalPose;

    // ---------------------------------- Controllers and LUTs ---------------------------------- //
    private InterpLUT wheelSpeed, hoodAngle;
    private SigmoidPositionProfile mp;
    private PIDFEx turretController, veloController; // TODO: Motion Profiling!!! Minimize
    public static double hood = 0.5, velo = 0.0;
    private PIDFExCoeffs coeffsTurret, coeffsVelo;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 1, 0);

    // ------------------------------------------ Util ------------------------------------------ //
    private Telemetry telemetry;
    private DoubleSupplier voltage;

    private double prev_target = 0;

    public static double kpTurret = 0.037, kiTurret = 0.08, kdTurret = 0.0018;

    public ShooterMP(RobotMap robotMap, Supplier<Pose> curPose, DecodeRobotV2.Alliance alliance,
                     Telemetry telemetry) {
        this.wheel1 = robotMap.getShooterWheel1Motor();
//        this.wheel2 = robotMap.getShooterWheel2Motor();
        this.hoodServo = robotMap.getHoodServo();
//        this.turretServoDriver = robotMap.getTurretServoDriver();
//        this.turretServoFollower = robotMap.getTurretServoFollower();
        this.turretMotor = robotMap.getTurretMotor();
        turretMotor.setInverted(true);
        turretMotor.resetEncoder();
//        this.turretServoPot = robotMap.getTurretServoPot();
//        this.telemetry = robotMap.getTelemetry();
        this.telemetry = telemetry;

        wheel1.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        wheel1.setInverted(true);
//        wheel2.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        // TODO

        // Select Correct Goal Based On Alliance
        goalPose = (alliance == DecodeRobotV2.Alliance.RED) ? REDGoalPose : BLUEGoalPose;

        mp = new SigmoidPositionProfile(0.8);
        coeffsTurret = new PIDFExCoeffs(
                0.037,
                0.08,
                0.0018,
                0.0,
                0.2,
                0.1,
                10,
                0.5
        );
        turretController = new PIDFEx(coeffsTurret);

        coeffsVelo = new PIDFExCoeffs(
                18,
                0.0,
                0.0,
                0.0,
                0.0,
                5,
                600,
                0.8
        );
        veloController = new PIDFEx(coeffsVelo);

        this.curPose = curPose;

        // Initialize LUTs here
        wheelSpeed = new InterpLUT();
        hoodAngle = new InterpLUT();

        // TODO: Populate LUTs with real data
        wheelSpeed.add(31.9, 0.74);
        wheelSpeed.add(38, 0.72);
        wheelSpeed.add(53.54, 0.76);
        wheelSpeed.add(67.41, 0.78);
        wheelSpeed.add(81.24, 0.84);
        wheelSpeed.add(97.3, 0.91);

        hoodAngle.add(31.9, 0.1);
        hoodAngle.add(38, 0.5);
        hoodAngle.add(53.54, 0.7);
        hoodAngle.add(67.41, 0.76);
        hoodAngle.add(81.24, 0.7);
        hoodAngle.add(97.3, 0.94);

        wheelSpeed.createLUT();
        hoodAngle.createLUT();

        voltage = () -> robotMap.getBattery().getVoltage();
    }

    @Override
    public void periodic() {

        // ------------------------------------- Telemetry -------------------------------------- //
        telemetry.addData("[Shooter] Wheel State ", wheelsEnabled);
        telemetry.addData("[Shooter] Turret Lock ", turretLockEnabled);
        telemetry.addData("[Shooter] Hood Lock ", hoodLockEnabled);
        telemetry.addData("[Shooter] Turret Angle: ", getTurretAngle());
        telemetry.addData("[Shooter] Goal Dist: ", getDistanceToGoal());
        telemetry.addData("[Shooter] Goal Angle: ", getAngleToGoal());
        telemetry.addData("[Shooter] Motor Current (A): ", ((DcMotorEx)wheel1.getRawMotor()).getCurrent(CurrentUnit.AMPS));
        telemetry.addData("[Shooter] Motor Power (W): ", voltage.getAsDouble()*velo*((DcMotorEx)wheel1.getRawMotor()).getCurrent(CurrentUnit.AMPS));

        // --------------------------------------- Turret --------------------------------------- //
//        turretController.setSetPoint(Range.clip(getAngleToGoal(), -40, 270)); // TODO: Tune Clips
//        if(getAngleToGoal() != prev_target) {
//            mp.reset(getTurretAngle(), getAngleToGoal());
//            prev_target = getAngleToGoal();
//        }
//        mp.update();

//        if(Math.abs(getAngleToGoal()-getTurretAngle()) < 5) {
            turretController.setSetPoint(Range.clip(getAngleToGoal(), -90, 225));
//        } else {
//            turretController.setSetPoint(Range.clip(mp.getPosition(), -225, 90));
//        }
        setTurretPower(Range.clip(
                turretController.calculate(getTurretAngle()),
                -MAX_TURRET_POWER,
                MAX_TURRET_POWER
        ));

//        if(getDistanceToGoal() < 31.9 || getDistanceToGoal() > 97.3) return; // TODO: LUT Bounds

        // ---------------------------------------- Hood ---------------------------------------- //
//        hoodServo.setPosition(Range.scale(
//                hoodAngle.get(getDistanceToGoal()), 0, 1, MIN_HOOD_POS, MAX_HOOD_POS)
//        );
        hoodServo.setPosition(Range.scale(hood, 0, 1, MIN_HOOD_POS, MAX_HOOD_POS));

        // --------------------------------------- Wheels --------------------------------------- //
        wheel1.set(getControlledWheelPower(velo));
//        wheel2.set(getControlledWheelPower(wheelSpeed.get(getDistanceToGoal())));
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
//        wheel2.set(0);
    }

    public boolean areWheelsEnabled() {
        return wheelsEnabled;
    }

    // ----------------------------------------- Turret ----------------------------------------- //
    public double getTurretAngle() {
        return (((turretMotor.getCurrentPosition())%TICKS_PER_FULL_ROTATION)*360.0/TICKS_PER_FULL_ROTATION)*1.0;
    }

//    private double analogToDegrees(double analogVal, double maxVoltage) {
//        return Range.scale(analogVal, 0.0, maxVoltage, 0, 360);
//    }
//
//    private double servoAngleToContinuousDegrees(double rawTurretServoAngle) {
//        if(rawTurretServoAngle != prevRawTurretServoAngle) {
//            if (Math.abs(rawTurretServoAngle - prevRawTurretServoAngle) >= 180)
//                turns += (rawTurretServoAngle > prevRawTurretServoAngle) ? -1 : 1;
//
//            prevRawTurretServoAngle = rawTurretServoAngle;
//        }
//        return rawTurretServoAngle + 360 * turns;
//    }

//    public double getTurretAngle() {
//        return servoAngleToContinuousDegrees(
//                analogToDegrees(
//                        turretServoPot.getVoltage(), turretServoPot.getMaxVoltage()
//                )
//        )*TURRET_RATIO*TURRET_MULTIPLIER;
//    }

    private void setTurretPower(double power) {
//        turretServoDriver.setPower(power);
//        turretServoFollower.setPower(-power);
        turretMotor.set(power);
    }

    // ---------------------------------------- IK Stuff ---------------------------------------- //
    private double getDistanceToGoal() {
        Pose pose = curPose.get();
        double dx = goalPose.getX() - pose.getX();
        double dy = goalPose.getY() - pose.getY();
        return Math.hypot(dx, dy);
    }

    public double getAngleToGoal() {
        Pose pose = curPose.get();
        double dx = goalPose.getX() - pose.getX();
        double dy = goalPose.getY() - pose.getY();
        return Math.toDegrees(Math.atan2(dy, dx)) - MathFunction.wrapDegrees(pose.getTheta());
    }
}
