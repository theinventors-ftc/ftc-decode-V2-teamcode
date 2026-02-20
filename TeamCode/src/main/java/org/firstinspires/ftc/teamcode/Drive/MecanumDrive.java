package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.Arrays;
import java.util.List;

@Config
public class MecanumDrive {
    static DriveConstants RobotConstants;
    static Telemetry telemetry;
    private MotorExEx frontLeft, frontRight, rearRight, rearLeft;
    private List<MotorExEx> motors;
    private com.arcrobotics.ftclib.drivebase.MecanumDrive drive;

    private boolean fieldCentricEnabled = true;

    private boolean autoEnabled = false;

    public MecanumDrive(
            RobotMap robotMap,
            DriveConstants robotConstants
    ) {
        this.RobotConstants = robotConstants;
        this.telemetry = robotMap.getTelemetry();

        this.frontLeft = robotMap.getFrontLeftMotor();
        this.frontRight = robotMap.getFrontRightMotor();
        this.rearLeft = robotMap.getRearLeftMotor();
        this.rearRight = robotMap.getRearRightMotor();

        motors = Arrays.asList(frontLeft, frontRight, rearLeft, rearRight);

        setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.BRAKE);

        setMotorsInverted(
                RobotConstants.frontLeftInverted,
                RobotConstants.frontRightInverted,
                RobotConstants.rearLeftInverted,
                RobotConstants.rearRightInverted
        );

        drive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(
                false, frontLeft, frontRight, rearLeft, rearRight
        );
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed,
                      double heading, double slow_input)
    {
        if(!autoEnabled){
            drive.setMaxSpeed(
                    RobotConstants.DEFAULT_SPEED_PERC - slow_input * RobotConstants.SLOW_SPEED_PERC
            );
            drive.driveFieldCentric(
                    strafeSpeed,
                    forwardSpeed,
                    turnSpeed,
                    fieldCentricEnabled ? heading : 0
            );
        } else {
            drive.setMaxSpeed(1);
            drive.driveRobotCentric(strafeSpeed,forwardSpeed, turnSpeed);
        }
    }

    public void setMotorsInverted(
            boolean frontLeftInv, boolean frontRightInv,
            boolean rearLeftInv, boolean rearRightInv
    )
    {
        frontLeft.setInverted(frontLeftInv);
        frontRight.setInverted(frontRightInv);
        rearLeft.setInverted(rearLeftInv);
        rearRight.setInverted(rearRightInv);
    }

    public void setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior zeroPowerBehavior)
    {
        for (MotorExEx motor : motors)
            motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public MotorExEx[] getMotors() {
        return new MotorExEx[]{frontLeft, frontRight, rearLeft, rearRight};
    }

    public void toggleMode() {
        fieldCentricEnabled = !fieldCentricEnabled;
    }

    public void setFieldCentric() {
        fieldCentricEnabled = true;
    }
    public void setRobotCentric() {
        fieldCentricEnabled = false;
    }

    public void setAutoEnabled(boolean enabled) {
        autoEnabled = enabled;
    }
}