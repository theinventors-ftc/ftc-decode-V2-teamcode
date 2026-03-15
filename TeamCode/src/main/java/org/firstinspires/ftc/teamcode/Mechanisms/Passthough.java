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

    // ---------------------------------------- States ------------------------------------------ //

    public enum FingerState {
        INTAKE,
        HOLD,
        FEED,
        REARRANGE,
        FLICK;

        double[][] positions = {
                {0.92, 0.975, 0.44, 0.86, 0.76}, // FRONT
                {0.9, 0.955, 0.42, 0.86, 0.75}, // CENTER
                {0.08, 0.045, 0.58, 0.13, 0.22}  // REAR
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

    private Map<Color, Double[][]> target_colors = Map.of( // [Color, Sensor(F, C, R), Value]
            Color.PURPLE, new Double[][]{
                    {0.378, 0.472, 0.771},
                    {0.283, 0.376, 0.518},
                    {0.397, 0.5, 0.763},
            },
            Color.GREEN, new Double[][]{
                    {0.2, 0.764, 0.591},
                    {0.146, 0.652, 0.412},
                    {0.22, 0.8, 0.605},
            }
    );
    private double NONE_artifact_thresh = 0.3;

    private Color[] current_colors = {
            Color.NONE,
            Color.NONE,
            Color.NONE
    };

    private Color[] contradiction_colors = {
            Color.NONE,
            Color.NONE,
            Color.NONE
    };

    private static final Map<MotifStorage.Motif, Color[]> MOTIF_MAP = Map.of(
            MotifStorage.Motif.PPG, new Color[]{Color.PURPLE, Color.PURPLE, Color.GREEN},
            MotifStorage.Motif.PGP, new Color[]{Color.PURPLE, Color.GREEN, Color.PURPLE},
            MotifStorage.Motif.GPP, new Color[]{Color.GREEN, Color.PURPLE, Color.PURPLE}
    );

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

        telemetry.addData("[Passthough] MOTIF: ", motif);
        telemetry.addData("[Passthough] FingerF State: ", getState(0));
        telemetry.addData("[Passthough] FingerC State: ", getState(1));
        telemetry.addData("[Passthough] FingerR State: ", getState(1));
        telemetry.addData("[Passthough] ColorF: ", getCurrentColor(0));
        telemetry.addData("[Passthough] ColorC: ", getCurrentColor(1));
        telemetry.addData("[Passthough] ColorR: ", getCurrentColor(2));
        telemetry.addData("[Passthough] ColorContradictedF: ", getContradictedColor(0));
        telemetry.addData("[Passthough] ColorContradictedC: ", getContradictedColor(1));
        telemetry.addData("[Passthough] ColorContradictedR: ", getContradictedColor(2));
        if(getShooting_order(0) != -1) {
            telemetry.addData("[Passthough] Shooting Order:", "%c, %c, %c",
                    names[getShooting_order(0)], names[getShooting_order(1)], names[getShooting_order(2)]);
        }
    }

    // ------------------------------------- Finger Control ------------------------------------- //
    public void setState(int finger, FingerState state) {
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

        telemetry.addData("[Passthough] ", "Finger %d:  %.3f", finger, distance);

        double greenDistance = Math.sqrt(
                Math.pow(colors[0] - target_colors.get(Color.GREEN)[finger][0], 2) +
                Math.pow(colors[1] - target_colors.get(Color.GREEN)[finger][1], 2) +
                Math.pow(colors[2] - target_colors.get(Color.GREEN)[finger][2], 2)
        );

        double purpleDistance = Math.sqrt(
                Math.pow(colors[0] - target_colors.get(Color.PURPLE)[finger][0], 2) +
                Math.pow(colors[1] - target_colors.get(Color.PURPLE)[finger][1], 2) +
                Math.pow(colors[2] - target_colors.get(Color.PURPLE)[finger][2], 2));

        if(greenDistance > NONE_artifact_thresh && purpleDistance > NONE_artifact_thresh) return Color.NONE;

        if(greenDistance > purpleDistance) return Color.PURPLE;

        return Color.GREEN;
    }

    public void updateColors() {
        current_colors[0] = detectColor(0);
        current_colors[1] = detectColor(1);
        current_colors[2] = detectColor(2);

        System.arraycopy(current_colors, 0, contradiction_colors, 0, 3);

        int purplesNeeded = 2;
        int greensNeeded = 1;

        for (Color c : current_colors) {
            if (c == Color.PURPLE) purplesNeeded--;
            if (c == Color.GREEN)  greensNeeded--;
        }

        if ((purplesNeeded + greensNeeded) != 3) {
            for (int i = 0; i < 3; i++) {
                if (contradiction_colors[i] == Color.NONE) {
                    if (purplesNeeded > 0) {
                        contradiction_colors[i] = Color.PURPLE;
                        purplesNeeded--;
                    } else if (greensNeeded > 0) {
                        contradiction_colors[i] = Color.GREEN;
                        greensNeeded--;
                    }
                }
            }
        }

        calculateShootingOrderMotif();
    }

    public Color getCurrentColor(int finger) {
        return current_colors[finger];
    }

    public Color getContradictedColor(int finger) {
        return contradiction_colors[finger];
    }

    public void calculateShootingOrderMotif() {
        this.shooting_order = new int[]{-1, -1, -1};

        int purple_count = 0, green_count = 0;
        for(Color color : contradiction_colors) {
            if(color == Color.PURPLE) purple_count++;
            else if(color == Color.GREEN) green_count++;
        }

        if(purple_count != 2 || green_count != 1) return;

        Color[] desired = MOTIF_MAP.get(motif);

        boolean[] used = new boolean[3];

        for (int i = 0; i < 3; i++) {
            for (int finger = 0; finger < 3; finger++) {
                if (!used[finger] && contradiction_colors[finger] == desired[i]) {
                    used[finger] = true;
                    shooting_order[i] = finger;
                    break;
                }
            }
        }
    }

    public int getShooting_order(int idx) {
        if(shooting_order[0] == -1) return idx;
        return shooting_order[idx];
    }

    public void updateMotif(MotifStorage.Motif motif) {
        this.motif = motif;
    }
}
