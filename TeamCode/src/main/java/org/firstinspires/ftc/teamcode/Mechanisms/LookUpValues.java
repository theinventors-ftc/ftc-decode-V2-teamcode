package org.firstinspires.ftc.teamcode.Mechanisms;

import com.arcrobotics.ftclib.util.InterpLUT;

public class LookUpValues {

    private InterpLUT wheelSpeed, hoodAngle;

    public enum CurrentWheel {
        BLUE_LIGHT,
        BRONZE_HEAVY
    }

    private CurrentWheel curWheel;

    public LookUpValues(CurrentWheel curWheel) {
        this.curWheel = curWheel;
        wheelSpeed = new InterpLUT();
        hoodAngle = new InterpLUT();
    }

    public void fiilWithValues() {
        if(curWheel == CurrentWheel.BLUE_LIGHT) {
            wheelSpeed.add(23.92697, 0.9);
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

            hoodAngle.add(23.92697, 0);
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
            return;
        }

        wheelSpeed.add(23.92697, 0.9);
        wheelSpeed.add(49.58, 0.552);
        wheelSpeed.add(62.66, 0.587);
        wheelSpeed.add(77.99, 0.616);
        wheelSpeed.add(86.54, 0.635);
        wheelSpeed.add(103.47, 0.696);
        wheelSpeed.add(118.3, 0.78);
        wheelSpeed.add(133.2, 0.832);
        wheelSpeed.add(142.94, 0.83);
        wheelSpeed.add(151.37, 0.856);
        wheelSpeed.add(164.46, 0.891);


        hoodAngle.add(23.92697, 0.0);
        hoodAngle.add(49.58, 0.0);
        hoodAngle.add(62.66, 0.17);
        hoodAngle.add(77.99, 0.23);
        hoodAngle.add(86.54, 0.31);
        hoodAngle.add(103.47, 0.41);
        hoodAngle.add(118.3, 0.61);
        hoodAngle.add(133.2, 0.7);
        hoodAngle.add(142.94, 0.63);
        hoodAngle.add(151.37, 0.7);
        hoodAngle.add(164.46, 0.75);

        wheelSpeed.createLUT();
        hoodAngle.createLUT();
    }

    public double getWheel(double dist) {
        return wheelSpeed.get(dist);
    }

    public double getHood(double dist) {
        return hoodAngle.get(dist);
    }

    public boolean inRange(double dist) {
        if(curWheel == CurrentWheel.BLUE_LIGHT) {
            return (dist > 48.4 && dist < 162.19);
        }

        return (dist > 49.59 && dist < 164.3);
    }
}
