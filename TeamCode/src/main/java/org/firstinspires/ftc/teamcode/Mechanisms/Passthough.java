package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.MotifStorage;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.Map;

@Config
public class Passthough extends SubsystemBase {
    // ---------------------------------------- Hardware ---------------------------------------- //
    private final ServoImplEx fingerF, fingerC, fingerR; // F: Front, C: Center, R: Rear
    private ColorSensor colorSensorF, colorSensorC, colorSensorR; // F: Front, C: Center, R: Rear

    private final ServoImplEx[] fingers;
    private final ColorSensor[] colorSensors;

    public static int gain = 50;

    // ---------------------------------------- States ------------------------------------------ //

    public enum FingerState {
        INTAKE,
        HOLD,
        FEED,
        REARRANGE;

        double[][] positions = {
                {0.94, 0.97, 0.42, 0.905}, // FRONT
                {0.9, 0.93, 0.38, 0.878}, // CENTER
                {0.09, 0.04, 0.6, 0.165}  // REAR
        };

        public double getPosition(int idx) {
            return positions[idx][this.ordinal()];
        }
    }

    private FingerState[] states = {
            FingerState.INTAKE,
            FingerState.INTAKE,
            FingerState.INTAKE
    };

    // ----------------------------------------- Colors ----------------------------------------- //

    public enum Color {
        PURPLE,
        GREEN,
        NONE
    }

    private Map<Color, Double[]> target_colors = Map.of(
            Color.PURPLE, new Double[]{0.504, 0.616, 0.961},
            Color.GREEN, new Double[]{0.246, 0.940, 0.714}
    );

    private Color[] current_colors = {
            Color.NONE,
            Color.NONE,
            Color.NONE
    };

    private static final Map<MotifStorage.Motif, Color[]> MOTIF_MAP = Map.of(
            MotifStorage.Motif.PPG, new Color[]{Color.PURPLE, Color.PURPLE, Color.GREEN},
            MotifStorage.Motif.PGP, new Color[]{Color.PURPLE, Color.GREEN, Color.PURPLE},
            MotifStorage.Motif.GPP, new Color[]{Color.GREEN, Color.PURPLE, Color.PURPLE}
    );

    private double distance_threshold = 30.0; // TODO: Chack if sensor hits a hole on the artifact

    // ----------------------------------------- Util ------------------------------------------- //
    private MotifStorage.Motif motif;
    private int[] shooting_order = {-1, -1, -1};

    private char[] names = {'F', 'C', 'R'};

    private Telemetry telemetry;

    public Passthough(RobotMap robotMap, MotifStorage.Motif motif) {
        this.motif = motif;

        this.fingerF = robotMap.getFingerFrontServo();
        this.fingerC = robotMap.getFingerCenterServo();
        this.fingerR = robotMap.getFingerRearServo();
        fingers = new ServoImplEx[]{fingerF, fingerC, fingerR};

        this.colorSensorF = robotMap.getColorSensorFront();
        this.colorSensorC = robotMap.getColorSensorCenter();
        this.colorSensorR = robotMap.getColorSensorRear();
        colorSensors = new ColorSensor[]{colorSensorF, colorSensorC, colorSensorR};

        this.telemetry = robotMap.getTelemetry();

        setState(0, FingerState.HOLD);
        setState(1, FingerState.HOLD);
        setState(2, FingerState.HOLD);
    }

    @Override
    public void periodic() {
//        updateCurrentColors(); // TODO: REMOVE IF TOO MUCH I2C TRAFFIC
//        shootingOrderMotif(); // TODO: REMOVE IF TOO MUCH CALCULATION
//
//        colorSensorR.setGain(gain);

        telemetry.addData("[Passthough] FingerF State: ", getState(0));
        telemetry.addData("[Passthough] FingerC State: ", getState(1));
        telemetry.addData("[Passthough] FingerR State: ", getState(1));
        telemetry.addData("[Passthough] ColorF: ", getCurrentColor(0));
        telemetry.addData("[Passthough] ColorC: ", getCurrentColor(1));
        telemetry.addData("[Passthough] ColorR: ", getCurrentColor(2));
        if(shooting_order[0] != -1) {
            telemetry.addData("[Passthough] Shooting Order:", "%c, %c, %c",
                    names[shooting_order[0]], names[shooting_order[1]], names[shooting_order[2]]);
        }
    }

    // ------------------------------------- Finger Control ------------------------------------- //
    public void setState(int finger, FingerState state) {
//        if(state == getState(finger)) return;
        states[finger] = state;
        fingers[finger].setPosition(state.getPosition(finger));
    }

    public FingerState getState(int finger) {
        return states[finger];
    }

    // ------------------------------------- Color Sensors -------------------------------------- //
    private Color detectColor(int finger) {
        double[] colors = colorSensors[finger].getRawColors();
        double distance = colorSensors[finger].getDistance(DistanceUnit.MM);

        telemetry.addData("[Passthough] ", "Finger %d:  %.3f, %.3f, %.3f", finger, colors[0], colors[1], colors[2]);
        telemetry.addData("[Passthough] ", "Finger %d:  %.3f", finger, distance);

        double minColorDistance = Double.MAX_VALUE;
        Color detectedColor = Color.NONE;

        if(distance < distance_threshold) {
            for (Color color : Color.values()) {
                if (color == Color.NONE) continue;
                Double[] target = target_colors.get(color);
                double colorDistance = Math.sqrt(
                        Math.pow(colors[0] - target[0], 2) +
                        Math.pow(colors[1] - target[1], 2) +
                        Math.pow(colors[2] - target[2], 2)
                );
                if (colorDistance < minColorDistance) {
                    minColorDistance = colorDistance;
                    detectedColor = color;
                }
            }

            return detectedColor;
        }

        return detectedColor;
    }

    public void updateCurrentColors() { // Call only when needed, minimize I2C traffic
        current_colors[0] = detectColor(0);
        current_colors[1] = detectColor(1);
        current_colors[2] = detectColor(2);
    }

    public Color getCurrentColor(int finger) {
        return current_colors[finger];
    }

    public void shootingOrderMotif() {
        this.shooting_order = new int[]{-1, -1, -1};

        int purple_count = 0, green_count = 0;
        for(Color color : current_colors) {
            if(color == Color.PURPLE) purple_count++;
            else if(color == Color.GREEN) green_count++;
        }

        if(purple_count != 2 || green_count != 1) return;

        Color[] desired = MOTIF_MAP.get(motif);

        boolean[] used = new boolean[3];

        // Match desired colors to fingers
        for (int i = 0; i < 3; i++) {
            for (int finger = 0; finger < 3; finger++) {
                if (!used[finger] && current_colors[finger] == desired[i]) {
                    used[finger] = true;
                    shooting_order[i] = finger + 0;
                    break;
                }
            }
        }
    }
}
