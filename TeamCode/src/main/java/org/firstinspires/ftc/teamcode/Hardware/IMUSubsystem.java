package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.function.DoubleSupplier;

public class IMUSubsystem extends SubsystemBase {
    private RevIMU imu;
    private DoubleSupplier pinpoint_yaw;

    private double previousRawYaw = 0;
    private double turns = 0;
    private double rawYaw = 0, rawPitch = 0, rawRoll = 0;
    private double contYaw;
    private double initYaw = 0;
    private Timer timer;
    private double in_time;

    private double[] angles;

    private Telemetry telemetry;

    public IMUSubsystem(RobotMap robotMap, double initYaw) {
        imu = new RevIMU(robotMap);
        imu.init(initYaw);
        timer = new Timer(0);
        timer.start();

        this.telemetry = robotMap.getTelemetry();
    }

    public IMUSubsystem(RobotMap robotMap, DoubleSupplier pinpoint) {
        pinpoint_yaw = pinpoint;
        timer = new Timer(0);
        timer.start();

        this.telemetry = robotMap.getTelemetry();
    }

    public void periodic() {
        in_time = timer.elapsedTime();

        if(imu != null) {
            angles = imu.getYawPitchRoll();
            rawYaw = angles[0];
            rawPitch = angles[1];
            rawRoll = angles[2];
        } else {
            rawYaw = pinpoint_yaw.getAsDouble();
            rawPitch = 0;
            rawRoll = 0;
        }

        calculateContinuousValue();
    }

    public double getYaw() {
        return contYaw;
    }

    public double getRawYaw() {
        return rawYaw+initYaw;
    }

    public double getPitch() {
        return rawPitch;
    }

    public double getRoll() {
        return rawRoll;
    }

    public int findClosestOrientationTarget() {
        int minDistIdx;
        int maxIdx = (int) Math.ceil(contYaw / 45);
        if (Math.abs((maxIdx - 1) * 45 - contYaw) > Math.abs((maxIdx) * 45 - contYaw))
            minDistIdx = maxIdx;
        else
            minDistIdx = maxIdx - 1;

        return minDistIdx * 45;
    }

    private void calculateContinuousValue() {
        if (Math.abs(rawYaw - previousRawYaw) >= 180)
            turns += (rawYaw > previousRawYaw) ? -1 : 1;

        previousRawYaw = rawYaw;
        contYaw = rawYaw + 360 * turns;
    }

    public void resetYawValue() {
        imu.resetYaw();
    }

    public void resetPinPointYawValue() {
        initYaw = -pinpoint_yaw.getAsDouble();
    }
}