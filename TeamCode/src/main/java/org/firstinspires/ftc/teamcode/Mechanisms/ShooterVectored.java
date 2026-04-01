package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
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

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

@Config
public class ShooterVectored extends SubsystemBase {
    // ---------------------------------------- Hardware ---------------------------------------- //
    private MotorExEx wheel1, wheel2;
    private ServoImplEx hoodServo;
    private MotorExEx turretMotor;

    // ---------------------------------------- Constants --------------------------------------- //
    // Wheel
    private static final int WHEEL_TICKS_PER_REV = 28, WHEEL_MAX_RPM = 5800;
    private static final double MAX_TICKS_PER_S = 2700; // WHEEL_MAX_RPM/60.0 * 28

    // Hood
    private static final double MIN_HOOD_POS = 0.9, MAX_HOOD_POS = 0.04;

    // Turret
    private static final double TICKS_PER_FULL_ROTATION = 1916.0;
    private static final double MAX_TURRET_POWER = 1.0;
    private static final double MIN_TURRET_ANGLE = -90.0, MAX_TURRET_ANGLE = 188.0;

    // ----------------------------------------- States ----------------------------------------- //
    private boolean wheelsEnabled = false;
    private boolean hoodLockEnabled = true;
    private boolean parking_state = false;

    public enum ShooterGoal {
        ALLIANCE_GOAL,
        OBELISK,
        DISABLED
    }

    private ShooterGoal shooterLock = ShooterGoal.DISABLED;

    // ---------------------------------------- Poses ------------------------------------------- //
    private Supplier<Pose> curPose, curPoseVel, futurePose;
    private final Pose REDGoalPose = new Pose(69.0, -68.0, 0);
    private final Pose BLUEGoalPose = new Pose(69.0, 68.0, 0);
    private final Pose ObeliskPose = new Pose(72.5, 0, 0);
    private final Pose goalPose;
    private DoubleSupplier tagX;

    // ---------------------------------- Controllers and LUTs ---------------------------------- //
    private InterpLUT wheelSpeed, hoodAngle;
    private PIDFEx turretController, veloController;
    private PIDFExCoeffs coeffsTurret, coeffsVelo;
    private MotorFF feedforward = new MotorFF(0.02, 1.02, 0.12);

    // ------------------------------------ Turret Zeroing -------------------------------------- //
    private boolean turretZeroed = false;
    private double turretZeroPower = -0.25;
    private double turretZeroCurrentThreshold = 2.0;
    public static double turretZeroOffset = 102.5;
    private StateMachine hasStalled;

    // ------------------------------------------ Util ------------------------------------------ //
    private Telemetry telemetry;
    private DoubleSupplier voltage;
    private ArrayList<Double> cachedDistances = new ArrayList<>();

    public static double customVEL = 0.0, customHOOD = 0.0, hoodOff = 0.48;
    private static final double stationaryScale = 1.0, robotVelocityScale = 0.00492, wheelSpeedFactor = 1.0;
    private static final double poseEstimation_dt = 0.21;


    public ShooterVectored(RobotMap robotMap, Supplier<Pose> curPose, Supplier<Pose> curPoseVel, DecodeRobotV2.Alliance alliance, boolean doZero) {
        this(robotMap, curPose, curPoseVel, alliance, doZero, () -> 320.0);
    }

    public ShooterVectored(RobotMap robotMap, Supplier<Pose> curPose, Supplier<Pose> curPoseVel,
                           DecodeRobotV2.Alliance alliance, boolean doZero, DoubleSupplier tagX) {
        this.wheel1 = robotMap.getShooterWheel1Motor();
        this.wheel2 = robotMap.getShooterWheel2Motor();
        this.hoodServo = robotMap.getHoodServo();
        this.turretMotor = robotMap.getTurretMotor();
        turretMotor.setInverted(true);
        turretMotor.resetEncoder();
        turretZeroed = !doZero;
        this.telemetry = robotMap.getTelemetry();
        this.tagX = tagX;
        this.curPose = curPose;
        this.futurePose = () -> estimateFuturePose(poseEstimation_dt);
        this.curPoseVel = curPoseVel;

        shooterLock = ShooterGoal.ALLIANCE_GOAL;

        hasStalled = new StateMachine(() -> ((DcMotorEx)turretMotor.getRawMotor()).getCurrent(CurrentUnit.AMPS) > turretZeroCurrentThreshold, 300);

        wheel1.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        wheel2.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        wheel2.setInverted(true);

        // Select Correct Goal Based On Alliance
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
                12.5, // 23
                0.0,
                0.04, // 0.09
                0.0,
                0.0,
                3,
                600,
                0.8
        );
        veloController = new PIDFEx(coeffsVelo);

        // Initialize LUTs here
        wheelSpeed = new InterpLUT();
        hoodAngle = new InterpLUT();

        wheelSpeed.add(48.4, 0.6);
        wheelSpeed.add(61.67, 0.605);
        wheelSpeed.add(79.75, 0.69);
        wheelSpeed.add(99.3, 0.74);
        wheelSpeed.add(114.68, 0.769);
        wheelSpeed.add(130.53, 0.865);
        wheelSpeed.add(133.67, 0.872);
        wheelSpeed.add(135.45, 0.9);
        wheelSpeed.add(142.89, 0.89);
        wheelSpeed.add(153.9, 0.901);
        wheelSpeed.add(162.2, 0.922);

        hoodAngle.add(48.4, 0);
        hoodAngle.add(61.67, 0);
        hoodAngle.add(79.75, 0.38);
        hoodAngle.add(99.3, 0.45);
        hoodAngle.add(114.68, 0.46);
        hoodAngle.add(130.53, 0.72);
        hoodAngle.add(133.67, 0.64);
        hoodAngle.add(135.45, 0.75);
        hoodAngle.add(142.89, 0.64);
        hoodAngle.add(153.9, 0.64);
        hoodAngle.add(162.2, 0.64);

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
        telemetry.addData("[Shooter] Parking State ", parking_state);
        telemetry.addData("[Shooter] Hood Lock ", hoodLockEnabled);
        telemetry.addData("[Shooter] Turret Angle: ", getTurretAngle());
        telemetry.addData("[Shooter] Turret Ticks: ", turretMotor.getCurrentPosition());
        telemetry.addData("[Shooter] GOAL Dist: ", getDistanceToGoal(curPose.get()));
        telemetry.addData("[Shooter] GOAL Angle: ", getAngleToGoal());

        // --------------------------------------- Turret --------------------------------------- //
        Vector curShootingVector = calcShootingVector();
        turretController.setSetPoint(getTurretTarget(curShootingVector));

        turretMotor.set(Range.clip(
                turretController.calculate(getTurretAngle()),
                -MAX_TURRET_POWER,
                MAX_TURRET_POWER
        ));

        if(!inLUTRange()) return;

        // ---------------------------------------- Hood ---------------------------------------- //
        hoodServo.setPosition(Range.scale(
                (hoodLockEnabled ? hoodAngle.get(getDistanceToGoal(futurePose.get())) : 0),
                0,
                1,
                MIN_HOOD_POS,
                MAX_HOOD_POS
        ));

        FtcDashboard.getInstance().getTelemetry().addData("Target Vel: ", 0.9 * customVEL * MAX_TICKS_PER_S);
        FtcDashboard.getInstance().getTelemetry().addData("Actual Vel: ", wheel1.getCorrectedVelocity());


        // --------------------------------------- Wheels --------------------------------------- //
        if(wheelsEnabled) {
            double futurePoseDist = getDistanceToGoal(futurePose.get());
            wheel1.set(getControlledWheelPower(wheelSpeed.get(futurePoseDist)));
            wheel2.set(getControlledWheelPower(wheelSpeed.get(futurePoseDist)));
//            wheel1.set(wheelSpeedLinear(futurePoseDist)*0.964);
//            wheel2.set(wheelSpeedLinear(futurePoseDist)*0.964);
        }
    }

    public double wheelSpeedLinear(double dist) {
        return 0.0026*dist + 0.50;
    }

    public void cacheCurrentDistance() {
        double dist = getDistanceToGoal(curPose.get());
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
        return turretMotor.getCurrentPosition()*(180.0/24209) - turretZeroOffset;
//        return (((turretMotor.getCurrentPosition())%TICKS_PER_FULL_ROTATION)*360.0/TICKS_PER_FULL_ROTATION)*(180.0/180.3797) - turretZeroOffset;
    }

    public void resetOffset() {
        turretZeroOffset = 0;
    }

    public boolean turretInRange() {
        double angleToGoal = getAngleToGoal();
        return angleToGoal > MIN_TURRET_ANGLE && angleToGoal < MAX_TURRET_ANGLE;
    }

    public boolean turretAtGoal() {
        return Math.abs(turretController.getPositionError()) < (atSmallTriangle() ? 1.2 : 0.7);
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

        Vector deltaPose = VectorMath.scale_vector(robotVelocityVec, dt);
        Vector futureVector = VectorMath.add_vectors(
                new Vector(
                    curPose.get().getX(),
                    curPose.get().getY(), false
                ),
                deltaPose
        );

        return new Pose(futureVector.getVx(), futureVector.getVy());
    }

    public Vector calcShootingVector() {
        if(!inLUTRange()) {
            return new Vector(0.6, Math.toRadians(getAngleToGoal()), true);
        }

        double stationary_angle = Math.toRadians(getAngleToGoal());
        double artifact_velocity = wheelSpeed.get(getDistanceToGoal(curPose.get()))*0.964;

        Vector stationaryVec = new Vector(artifact_velocity, stationary_angle, true);
        Vector robotVelocityVec = new Vector(curPoseVel.get().getX(), curPoseVel.get().getY(), false);

        Vector scaledStationaryVec = VectorMath.scale_vector(stationaryVec, stationaryScale);
        Vector scaledRobotVelocityVec = VectorMath.scale_vector(robotVelocityVec, robotVelocityScale);

        telemetry.addData("[Shooter] Stationary Vector (r, θ): ", "(%.2f, %.2f)", stationaryVec.getMagnitude(), Math.toDegrees(stationaryVec.getAngle()));
        telemetry.addData("[Shooter] Velocity Vector (x, y): ", "(%.2f, %.2f)", robotVelocityVec.getVx(), robotVelocityVec.getVy());
        telemetry.addData("[Shooter] Velocity Vector (r, θ): ", "(%.2f, %.2f)", robotVelocityVec.getMagnitude(), Math.toDegrees(robotVelocityVec.getAngle()));

        if(robotVelocityVec.getMagnitude() < 2.0) return stationaryVec;

        return VectorMath.scale_vector(
                VectorMath.subtract_vectors(scaledStationaryVec, scaledRobotVelocityVec),
                wheelSpeedFactor
        );
    }

    public double getTagTargetX() {
        return 320.0;
    }

    public boolean inLUTRange() {
        double dist = getDistanceToGoal(curPose.get());
        double distFutur = getDistanceToGoal(futurePose.get());
        return (dist > 48.4 && dist < 162.19) && (distFutur > 48.4 && distFutur < 162.19);
    }

    public boolean atSmallTriangle() {
        return getDistanceToGoal(curPose.get()) > (325.0/2.54);
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

    public void enableObelisk() {
        //pare mou mia pipa
    }

    public void disableObelisk() {
        //pare mou mia pipa
    }
}
