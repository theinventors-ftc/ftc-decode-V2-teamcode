package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MotifStorage;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.List;
import java.util.Map;

@Config
public class Detection extends SubsystemBase {
    private final Limelight3A limelight;

    private int motifId = 0;

    public enum DetectionState {
        OBELISK(1),
        GOAL(2),
        DISABLED;

        public final int pipeline;

        DetectionState(int pipeline) {
            this.pipeline = pipeline;
        }

        DetectionState() {
            this.pipeline = -1;
        }
    }

    private DetectionState state = DetectionState.DISABLED;
    private Telemetry telemetry;

    public Detection(RobotMap robotMap) {
        this.limelight = robotMap.getLimelight();
        this.telemetry = robotMap.getTelemetry();

        setState(DetectionState.OBELISK);
    }

    @Override
    public void periodic() {
        if(state == DetectionState.DISABLED) return;

        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

//            Pose3D botpose = result.getBotpose();
//            telemetry.addData("LIME POSE", "X: %.2f, Y: %.2f, Z: %.2f", botpose.getPosition().x*39.370078, botpose.getPosition().y*39.370078, botpose.getPosition().z);

            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            if (fiducialResults.isEmpty()) {
                motifId = 0;
                telemetry.addData("Limelight", "No tag available");
                return;
            }

            motifId = fiducialResults.get(0).getFiducialId();

        } else {
            telemetry.addData("Limelight", "No data available");
            motifId = -1;
        }

        telemetry.addData("Limelight Motif Id: ", motifId);
    }

    public void setState(DetectionState state) {
        if(this.state == DetectionState.DISABLED && state != DetectionState.DISABLED) {
            limelight.start();
        }

        this.state = state;

        if(state == DetectionState.DISABLED) {
            limelight.pause();
            return;
        }

        limelight.pipelineSwitch(state.pipeline);
    }

    public Map<Integer, MotifStorage.Motif> motifPoses = Map.of(
            21, MotifStorage.Motif.GPP,
            22, MotifStorage.Motif.PGP,
            23, MotifStorage.Motif.PPG
    );

    public MotifStorage.Motif getMotif() {
        return motifPoses.getOrDefault(motifId, MotifStorage.Motif.GPP);
    }

    public void setGoalPip() {
        setState(DetectionState.GOAL);
    }
}
