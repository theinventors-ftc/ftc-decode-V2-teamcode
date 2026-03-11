package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Battery;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Hardware.GamepadExEx;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.Localization.GoBildaPinpointDriver;

import java.util.List;

public class RobotMap {
    private HardwareMap hm;

    private GamepadExEx driverOp, toolOp;

    private MotorExEx frontLeft, rearLeft, frontRight, rearRight;

    private GoBildaPinpointDriver odo = null;
    private GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection = null, forwardEncoderDirection = null;
    private GoBildaPinpointDriver.GoBildaOdometryPods encoderRes = null;


    private List<LynxModule> hubs;
    private Telemetry telemetry;
    private Battery battery;
    private Limelight3A limelight;

    //// Mechanisms
    //Intake
    private MotorExEx intakeMotor;

    // Passthough
    private ServoImplEx fingerF, fingerC, fingerR;
    private ColorSensor colorSensorF, colorSensorC, colorSensorR;

    // Shooter
    private MotorExEx wheel1, wheel2, turret;
    private ServoImplEx hoodServo;

    public RobotMap(HardwareMap hm, Telemetry telemetry) {
        this(hm, telemetry, null, null);
    }

    public RobotMap (HardwareMap hm, Telemetry telemetry, Gamepad driverOp,
                     Gamepad toolOp) {
        this.hm = hm;

        this.telemetry = telemetry;
        this.telemetry.setMsTransmissionInterval(11);

        if(driverOp != null) this.driverOp = new GamepadExEx(driverOp);
        if(toolOp != null) this.toolOp = new GamepadExEx(toolOp);

        hubs = hm.getAll(LynxModule.class);
        battery = new Battery(hm);

        /*--Motors--*/
        frontLeft = new MotorExEx(hm, "frontLeft", Motor.GoBILDA.RPM_1150);
        rearLeft = new MotorExEx(hm, "rearLeft", Motor.GoBILDA.RPM_1150);
        frontRight = new MotorExEx(hm, "frontRight", Motor.GoBILDA.RPM_1150);
        rearRight = new MotorExEx(hm, "rearRight", Motor.GoBILDA.RPM_1150);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        rearLeft.setRunMode(Motor.RunMode.RawPower);
        rearRight.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        /*--Encoders--*/
        if(driverOp != null || toolOp!=null) {
            odo = hm.get(GoBildaPinpointDriver.class, "odometry");
            encoderRes = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
            forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
            strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        }

        /*--Util--*/
        for (LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

//        limelight = hm.get(Limelight3A.class, "limelight");

        //// ----------------------------------- Mechanisms ----------------------------------- ////
        // Intake
        intakeMotor = new MotorExEx(hm, "intakeMotor", Motor.GoBILDA.RPM_1150);

        // Passthrough
        fingerF = hm.get(ServoImplEx.class, "fingerF");
        fingerC = hm.get(ServoImplEx.class, "fingerC");
        fingerR = hm.get(ServoImplEx.class, "fingerR");
        colorSensorF = new ColorSensor(hm, "colorSensorF");
        colorSensorC = new ColorSensor(hm, "colorSensorC");
        colorSensorR = new ColorSensor(hm, "colorSensorR");
        colorSensorF.setGain(10);
        colorSensorC.setGain(10);
        colorSensorR.setGain(10);

        // Shooter
        turret = new MotorExEx(hm, "turretMotor", Motor.GoBILDA.RPM_435);
        wheel1 = new MotorExEx(hm, "wheel1", Motor.GoBILDA.BARE);
        wheel2 = new MotorExEx(hm, "wheel2", Motor.GoBILDA.BARE);
        hoodServo = hm.get(ServoImplEx.class, "hoodServo");
    }

    // ---------------------------------------- Gamepads ---------------------------------------- //
    public GamepadExEx getDriverOp() {
        return driverOp;
    }

    public GamepadExEx getToolOp() {
        return toolOp;
    }

    // ----------------------------------------- Motors ----------------------------------------- //
    public MotorExEx getFrontLeftMotor() {
        return frontLeft;
    }

    public MotorExEx getFrontRightMotor() {
        return frontRight;
    }

    public MotorExEx getRearLeftMotor() {
        return rearLeft;
    }

    public MotorExEx getRearRightMotor() {
        return rearRight;
    }

    // ---------------------------------------- Encoders ---------------------------------------- //
    public GoBildaPinpointDriver.GoBildaOdometryPods getEncoderRes() {
        return encoderRes;
    }

    public GoBildaPinpointDriver getOdometry() {
        return odo;
    }

    public GoBildaPinpointDriver.EncoderDirection getStrafeEncoderDirection() {
        return strafeEncoderDirection;
    }

    public GoBildaPinpointDriver.EncoderDirection getForwardEncoderDirection() {
        return forwardEncoderDirection;
    }

    // ------------------------------------------ Util ------------------------------------------ //
    public Telemetry getTelemetry() {
        return telemetry;
    }
    public List<LynxModule> getHubs() {
        return hubs;
    }
    public Battery getBattery() { return battery; }
    public Limelight3A getLimelight() {
        return limelight;
    }

    // ------------------------------------------ IMU ------------------------------------------- //
    public IMU getIMU() {
        return null;
    }

    //// ------------------------------------- Mechanisms ------------------------------------- ////
    // Intake
    public MotorExEx getIntakeMotor() {
        return intakeMotor;
    }

    // Passthrough
    public ServoImplEx getFingerFrontServo() {
        return fingerF;
    }

    public ServoImplEx getFingerCenterServo() {
        return fingerC;
    }

    public ServoImplEx getFingerRearServo() {
        return fingerR;
    }

    public ColorSensor getColorSensorFront() {
        return colorSensorF;
    }

    public ColorSensor getColorSensorCenter() {
        return colorSensorC;
    }

    public ColorSensor getColorSensorRear() {
        return colorSensorR;
    }

    // Shooter
    public MotorExEx getShooterWheel1Motor() {
        return wheel1;
    }

    public MotorExEx getShooterWheel2Motor() {
        return wheel2;
    }

    public ServoImplEx getHoodServo() {
        return hoodServo;
    }

    public MotorExEx getTurretMotor() {
        return turret;
    }
}
