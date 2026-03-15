package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.function.DoubleSupplier;

public class PinpointYawWrapper extends SubsystemBase {
    private DoubleSupplier pinpoint_yaw;

    private double previousRawYaw = 0;
    private double turns = 0;
    private double rawYaw = 0;
    private double contYaw;
    private double initYaw = 0;
    private Timer timer;

    public PinpointYawWrapper(RobotMap robotMap, DoubleSupplier pinpoint) {
        pinpoint_yaw = pinpoint;
        timer = new Timer(0);
        timer.start();
    }

    public void periodic() {
        rawYaw = pinpoint_yaw.getAsDouble();

        if (Math.abs(rawYaw - previousRawYaw) >= 180)
            turns += (rawYaw > previousRawYaw) ? -1 : 1;

        previousRawYaw = rawYaw;
        contYaw = rawYaw + 360 * turns;
    }

    public double getContinuousYaw() {
        return contYaw;
    }

    public double getRawYaw() {
        return rawYaw+initYaw;
    }

    public void resetYawValue() {
        initYaw = -pinpoint_yaw.getAsDouble();
    }
}