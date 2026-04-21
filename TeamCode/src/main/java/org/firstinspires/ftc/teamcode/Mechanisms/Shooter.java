package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Controllers.MotorFF;
import org.firstinspires.ftc.teamcode.Controllers.PIDFEx;
import org.firstinspires.ftc.teamcode.Controllers.PIDFExCoeffs;
import org.firstinspires.ftc.teamcode.Controllers.StateMachine;
import org.firstinspires.ftc.teamcode.DecodeRobotV2;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.RobotMap;

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
    private static final double MAX_TICKS_PER_S = 2700;

    // Hood
    private static final double MIN_HOOD_POS = 0.9, MAX_HOOD_POS = 0.04;

    // Turret
    private static final double TICKS_PER_FULL_ROTATION = 48418.0;
    private static final double MAX_TURRET_POWER = 1.0;
    private static final double MIN_TURRET_ANGLE = -90.0, MAX_TURRET_ANGLE = 188.0;

    // IK (Vectoring)
    private static final double
        stationaryScale = 1.0,
        robotVelocityScale = 0.00492,
        robotVelocityScaleAuto = 0.003,
        wheelSpeedFactor = 1.0;

    private static final double poseEstimation_dt = 0.21;

    // ----------------------------------------- States ----------------------------------------- //
    private boolean wheelsEnabled = false;
    private boolean hoodLockEnabled = true;

    public enum ShooterGoal {
        ALLIANCE_GOAL,
        OBELISK,
        AUTO_CUSTOM,
        DISABLED
    }

    private ShooterGoal shooterLock;

    // ---------------------------------------- Poses ------------------------------------------- //
    private Supplier<Pose> curPose, curPoseVel, futurePose, curPoseAccel;
    private final Pose REDGoalPose = new Pose(69.0, -67.0, 0);
    private final Pose BLUEGoalPose = new Pose(69.0, 67.0, 0);
    private final Pose ObeliskPose = new Pose(72.5, 0, 0);
    private final Pose goalPose;

    // ---------------------------------- Controllers and LUTs ---------------------------------- //
    private LookUpValues lu_values;
    private PIDFEx turretController, veloController;
    private PIDFExCoeffs coeffsTurret, coeffsVelo;
    private MotorFF feedforward = new MotorFF(0.02, 1.02, 0.12);
    private boolean inAuto = false;

    // ------------------------------------ Turret Zeroing -------------------------------------- //
    private boolean turretZeroed = false;
    private double turretZeroPower = -0.25;
    private double turretZeroCurrentThreshold = 2.0;
    private static double turretZeroOffset = 102.5;
    private static double turretZeroOffsetReversed = -195.5346;
    private boolean startReversed = false;
    private StateMachine hasStalled;

    private double auto_custom_angle = 0.0;

    private boolean accelWheel = false;

    // ------------------------------------------ Util ------------------------------------------ //
    private Telemetry telemetry;
    private DoubleSupplier voltage;

    public static double custom_vel = 0.2, custom_hood = 0.0;

    public Shooter(RobotMap robotMap, Supplier<Pose> curPose, Supplier<Pose> curPoseVel,
                   DecodeRobotV2.Alliance alliance, boolean doZero) {

        this(robotMap, curPose, curPoseVel, () -> new Pose(0,0,0), alliance, doZero, false, false);
    }

    public Shooter(RobotMap robotMap, Supplier<Pose> curPose, Supplier<Pose> curPoseVel, Supplier<Pose> curPoseAccel,
                   DecodeRobotV2.Alliance alliance, boolean doZero, boolean startReversed,
                   boolean inAuto
    ) {
        this(robotMap, curPose, curPoseVel, curPoseAccel, alliance, doZero, startReversed, inAuto, false);
    }

    public Shooter(RobotMap robotMap, Supplier<Pose> curPose, Supplier<Pose> curPoseVel, Supplier<Pose> curPoseAccel,
                   DecodeRobotV2.Alliance alliance, boolean doZero, boolean startReversed,
                   boolean inAuto, boolean accelWheel
    ) {
        this.wheel1 = robotMap.getShooterWheel1Motor();
        this.wheel2 = robotMap.getShooterWheel2Motor();

        this.hoodServo = robotMap.getHoodServo();

        this.turretMotor = robotMap.getTurretMotor();
        turretMotor.setInverted(true);
        turretMotor.resetEncoder();
        turretZeroed = !doZero;
        this.startReversed = !doZero && startReversed;
        this.inAuto = inAuto;
        this.accelWheel = accelWheel;

        this.telemetry = robotMap.getTelemetry();

        this.curPose = curPose;
        this.futurePose = () -> estimateFuturePose(poseEstimation_dt);
        this.curPoseVel = curPoseVel;
        this.curPoseAccel = curPoseAccel;

        shooterLock = ShooterGoal.ALLIANCE_GOAL;

        hasStalled = new StateMachine(() -> ((DcMotorEx)turretMotor.getRawMotor()).getCurrent(CurrentUnit.AMPS) > turretZeroCurrentThreshold, 300);

        wheel1.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        wheel2.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        wheel2.setInverted(true);

        goalPose = (alliance == DecodeRobotV2.Alliance.RED) ? REDGoalPose : BLUEGoalPose;

        coeffsTurret = new PIDFExCoeffs( //Salonika: kP=0.072, kI=0.16, kD=0.0018,
                0.055,
                0.13,
                0.00225,
                0.0,
                0.2,
                0.0,
                20,
                0.6
        );

        turretController = new PIDFEx(coeffsTurret);

        coeffsVelo = new PIDFExCoeffs(
                20, // 23
                0.0,
                0.05, // 0.09
                0.0,
                0.0,
                3,
                600,
                0.8
        );
        veloController = new PIDFEx(coeffsVelo);

        lu_values = new LookUpValues(LookUpValues.CurrentWheel.BRONZE_HEAVY);
        lu_values.fiilWithValues();

//        wheelSpeed.add(23.92697, 0.8);
//        wheelSpeed.add(48.4, 0.6);
//        wheelSpeed.add(61.67, 0.605);
//        wheelSpeed.add(79.75, 0.69);
//        wheelSpeed.add(99.3, 0.74);
//        wheelSpeed.add(114.68, 0.769);
//        wheelSpeed.add(130.53, 0.865);
//        wheelSpeed.add(133.67, 0.872);
//        wheelSpeed.add(135.45, 0.9);
//        wheelSpeed.add(142.89, 0.89);
//        wheelSpeed.add(153.9, 0.901);
//        wheelSpeed.add(162.2, 0.922);
//
//        hoodAngle.add(23.92697, 0);
//        hoodAngle.add(48.4, 0);
//        hoodAngle.add(61.67, 0);
//        hoodAngle.add(79.75, 0.38);
//        hoodAngle.add(99.3, 0.45);
//        hoodAngle.add(114.68, 0.46);
//        hoodAngle.add(130.53, 0.72);
//        hoodAngle.add(133.67, 0.64);
//        hoodAngle.add(135.45, 0.75);
//        hoodAngle.add(142.89, 0.64);
//        hoodAngle.add(153.9, 0.64);
//        hoodAngle.add(162.2, 0.64);

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

        if(shooterLock == ShooterGoal.DISABLED) {
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
        telemetry.addData("[Shooter] Hood Lock ", hoodLockEnabled);
        telemetry.addData("[Shooter] Shooter ", shooterLock);
        telemetry.addData("[Shooter] Turret Angle: ", getTurretAngle());
        telemetry.addData("[Shooter] Turret Ticks: ", turretMotor.getCurrentPosition());
        telemetry.addData("[Shooter] GOAL Dist: ", getDistanceToGoal(futurePose.get()));
        telemetry.addData("[Shooter] GOAL Angle: ", getAngleToGoal());

        // --------------------------------------- Turret --------------------------------------- //
        Vector curShootingVector = calcShootingVector();
        if(shooterLock == ShooterGoal.ALLIANCE_GOAL) {
            turretController.setSetPoint(wheelsEnabled ? getTurretTarget(curShootingVector) : 0);
        }

        if(shooterLock == ShooterGoal.AUTO_CUSTOM) {
            turretController.setSetPoint(auto_custom_angle);
        }

        turretMotor.set(Range.clip(
                turretController.calculate(getTurretAngle()),
                -MAX_TURRET_POWER,
                MAX_TURRET_POWER
        ));

        if(accelWheel && !accelVelTarget()) {
            wheel1.set(1.0);
            wheel2.set(1.0);
            accelWheel = false;
        }

        if(!inLUTRange()) return;

        // ---------------------------------------- Hood ---------------------------------------- //
        hoodServo.setPosition(Range.scale(
                (hoodLockEnabled ? lu_values.getHood(getDistanceToGoal(futurePose.get())) : 0),
                0,
                1,
                MIN_HOOD_POS,
                MAX_HOOD_POS
        ));

//        hoodServo.setPosition(Range.scale(
//                (hoodLockEnabled ? custom_hood : 0),
//                0,
//                1,
//                MIN_HOOD_POS,
//                MAX_HOOD_POS
//        ));

        // --------------------------------------- Wheels --------------------------------------- //
        if(wheelsEnabled) {
            double futurePoseDist = getDistanceToGoal(futurePose.get());
            wheel1.set(getControlledWheelPower(lu_values.getWheel(futurePoseDist)));
            wheel2.set(getControlledWheelPower(lu_values.getWheel(futurePoseDist)));
//            wheel1.set(getControlledWheelPower(custom_vel));
//            wheel2.set(getControlledWheelPower(custom_vel));
        }
    }

    // ----------------------------------------- Wheels ----------------------------------------- //
    public double getControlledWheelPower(double power) {
        double speed = 0.9 * power * MAX_TICKS_PER_S;
        veloController.setSetPoint(speed);
        double velocity = veloController.calculate(wheel1.getCorrectedVelocity()) +
                feedforward.calculate(speed, wheel1.getAcceleration());
        return velocity / MAX_TICKS_PER_S;
    }

    public boolean accelVelTarget() {
        return wheel1.getCorrectedVelocity() > 1750;
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
        if (startReversed) {
            return turretMotor.getCurrentPosition()*(360.0/TICKS_PER_FULL_ROTATION) - turretZeroOffsetReversed;
        }

        return turretMotor.getCurrentPosition()*(360.0/TICKS_PER_FULL_ROTATION) - turretZeroOffset;
    }

    public void resetOffset() {
        turretZeroOffset = startReversed ? 102.5 : 195.5346;
    }

    public boolean turretInRange() {
        double angleToGoal = getAngleToGoal();
        return angleToGoal > MIN_TURRET_ANGLE && angleToGoal < MAX_TURRET_ANGLE;
    }

    public boolean turretAtGoal() {
        return Math.abs(turretController.getPositionError()) < (atSmallTriangle() ? 1.2 : 0.7);
    }

    public boolean turretAtGoal(double threshold) {
        return Math.abs(turretController.getPositionError()) < threshold;
    }

    // ---------------------------------------- IK Stuff ---------------------------------------- //
    public double getDistanceToGoal(Pose posaki) {
        double dx = goalPose.getX() - posaki.getX();
        double dy = goalPose.getY() - posaki.getY();
        return Math.hypot(dx, dy);
    }

    public double getAngleToGoal() { // true -> goal, false -> obelisk
        double dx_ref = goalPose.getX() - curPose.get().getX();
        double dy_ref = goalPose.getY() - curPose.get().getY();

        if(shooterLock == ShooterGoal.OBELISK) {
            dx_ref = ObeliskPose.getX() - curPose.get().getX();
            dy_ref = ObeliskPose.getY() - curPose.get().getY();
        }

        double targetAngle_ref = Math.toDegrees(Math.atan2(dy_ref, dx_ref));
        double dx = dx_ref;
        double dy = dy_ref;

        if(Math.abs(targetAngle_ref) > 70.0) dx += Range.scale(targetAngle_ref, -70.0, -90.0, -1.5, -0.8);
        if(atSmallTriangle()) dy += 0;

        return Math.toDegrees(Math.atan2(dy, dx));
    }

    public double getTurretTarget(Vector shooting) {
        double robotHeading = curPose.get().getTheta() % 360;
        if (robotHeading >= 180) robotHeading -= 360;
        if (robotHeading < -180) robotHeading += 360;

        double relativeAngle = Math.toDegrees(shooting.getAngle()) - robotHeading;
        relativeAngle %= 360;
        if (relativeAngle >= 180) relativeAngle -= 360;
        if (relativeAngle < -180) relativeAngle += 360;

        relativeAngle = Range.clip(relativeAngle, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);

        return relativeAngle;
    }

    public Pose estimateFuturePose(double dt) {
        Vector robotVelocityVec = new Vector(curPoseVel.get().getX(), curPoseVel.get().getY(), false);

        if(robotVelocityVec.getMagnitude() < 0.5) return curPose.get();

        Vector robotAccelVec = new Vector(curPoseAccel.get().getX(), curPoseAccel.get().getY(), false);

        Vector Ut = VectorMath.scale_vector(robotVelocityVec, dt);

        Vector At = VectorMath.scale_vector(robotAccelVec, 0.5 * (dt * dt));

        Vector futureVector = VectorMath.add_vectors(
                new Vector(
                    curPose.get().getX(),
                    curPose.get().getY(), false
                ),
                Ut,
                At
        );

        return new Pose(futureVector.getVx(), futureVector.getVy());
    }

    public Vector calcShootingVector() {
        if(!inLUTRange()) {
            return new Vector(1, Math.toRadians(getAngleToGoal()), true);
        }

        double stationary_angle = Math.toRadians(getAngleToGoal());
        double artifact_velocity = lu_values.getWheel(getDistanceToGoal(futurePose.get())); // piapaaa

        Vector stationaryVec = new Vector(artifact_velocity, stationary_angle, true);
        Vector robotVelocityVec = new Vector(curPoseVel.get().getX(), curPoseVel.get().getY(), false);

        Vector scaledStationaryVec = VectorMath.scale_vector(stationaryVec, stationaryScale);

        Vector scaledRobotVelocityVec;

        if (!inAuto) {
            scaledRobotVelocityVec = VectorMath.scale_vector(robotVelocityVec, robotVelocityScale);
        } else {
            scaledRobotVelocityVec = VectorMath.scale_vector(robotVelocityVec, robotVelocityScaleAuto);
        }

        telemetry.addData("[Shooter] Stationary Vector (r, θ): ", "(%.2f, %.2f)", stationaryVec.getMagnitude(), Math.toDegrees(stationaryVec.getAngle()));
        telemetry.addData("[Shooter] Velocity Vector (x, y): ", "(%.2f, %.2f)", robotVelocityVec.getVx(), robotVelocityVec.getVy());
        telemetry.addData("[Shooter] Velocity Vector (r, θ): ", "(%.2f, %.2f)", robotVelocityVec.getMagnitude(), Math.toDegrees(robotVelocityVec.getAngle()));

        if(robotVelocityVec.getMagnitude() < 2.0) return stationaryVec;

        return VectorMath.scale_vector(
                VectorMath.subtract_vectors(scaledStationaryVec, scaledRobotVelocityVec),
                wheelSpeedFactor
        );
    }

    public boolean inLUTRange() {
        return lu_values.inRange(getDistanceToGoal(futurePose.get()));
    }

    public boolean atSmallTriangle() {
        return getDistanceToGoal(futurePose.get()) > (325.0/2.54);
    }

    public void zeroTurret() {
        turretZeroed = false;
    }

    public void enableParkingState() {
        shooterLock = ShooterGoal.DISABLED;
    }

    public void disableParkingState() {
        shooterLock = ShooterGoal.ALLIANCE_GOAL;
    }

    public boolean isParked(){
        return shooterLock == ShooterGoal.DISABLED;
    }

    public void increase_turret_offset() { turretZeroOffset += 1.0; }
    public void decrease_turret_offset() { turretZeroOffset -= 1.0; }

    public void enableObelisk() {
        //pare mou mia pipa
    }

    public void disableObelisk() {
        //pare mou mia pipa
    }

    public void enableAutoCustom(double customAngle) {
        shooterLock = ShooterGoal.AUTO_CUSTOM;
        auto_custom_angle = customAngle;
    }
}
