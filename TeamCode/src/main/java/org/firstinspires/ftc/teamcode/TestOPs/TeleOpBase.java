package org.firstinspires.ftc.teamcode.TestOPs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DecodeRobotV2;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Hardware.GamepadExEx;
import org.firstinspires.ftc.teamcode.MotifStorage;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Util.Timer;

@Disabled
@TeleOp(name = "Do not run this TeleOP", group = "")
public class TeleOpBase extends CommandOpMode {
    private DriveConstants RobotConstants;
    private ElapsedTime runtime;

    private Timer loopTime;
    private DecodeRobotV2 robot;

    private RobotMap robotMap;
    private Pose pose = new Pose(0,0,0);

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset(); // Ultra SOS
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2);
        loopTime = new Timer();

        // ----------------------------------- Robot Constants ---------------------------------- //
        RobotConstants = new DriveConstants();

        RobotConstants.frontLeftInverted = false;
        RobotConstants.frontRightInverted = true;
        RobotConstants.rearRightInverted = true;
        RobotConstants.rearLeftInverted = false;

        RobotConstants.DEFAULT_SPEED_PERC = 1.0;
        RobotConstants.SLOW_SPEED_PERC = 0.7;

        // ---------------------------- Transfer Pose from Autonomous --------------------------- //
        pose = PoseStorage.currentPose;
    }

    public void initAllianceRelated(DecodeRobotV2.Alliance alliance) {
        robot = new DecodeRobotV2(
            robotMap,
            RobotConstants,
            alliance,
            pose,
            MotifStorage.currentMotif
        );
    }

    @Override
    public void run() {
        super.run();

        for (LynxModule hub : robotMap.getHubs()) hub.clearBulkCache();

        robot.drive_update();

        telemetry.addData("Loop Hz: ", 1.0/loopTime.getElapsedTimeSeconds());
        loopTime.resetTimer();
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public void reset() {
        super.reset();
        PoseStorage.currentPose = robot.getPose(); // In case we stop TeleOP midway
//        robotMap.getLimelight().close();
    }
}