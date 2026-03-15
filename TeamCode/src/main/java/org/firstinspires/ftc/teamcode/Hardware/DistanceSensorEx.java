package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorEx {
    DistanceSensor sensor;

    public DistanceSensorEx(HardwareMap hm, String id) {
        sensor = hm.get(DistanceSensor.class, id);
    }

    public double getDistance() {
        return sensor.getDistance(DistanceUnit.CM);
    }

    public double getDistance(DistanceUnit unit) {
        return sensor.getDistance(unit);
    }
}
