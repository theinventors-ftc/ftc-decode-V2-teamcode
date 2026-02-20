package org.firstinspires.ftc.teamcode.TestOPs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DecodeRobot;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Hardware.GamepadExEx;
import org.firstinspires.ftc.teamcode.MotifStorage;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.RobotMap;

@Disabled
@TeleOp(name = "Do not run this TeleOP", group = "")
public class TeleOpBase extends CommandOpMode {
    GamepadExEx driverOp, toolOp;
    private DriveConstants RobotConstants;
    private ElapsedTime runtime;
    private DecodeRobot robot;

    private RobotMap robotMap;
    private Pose pose = new Pose(-72 + 8.375,8.5,0);

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset(); // Ultra SOS
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2);

        // ----------------------------------- Robot Constants ---------------------------------- //
        RobotConstants = new DriveConstants();

        RobotConstants.frontLeftInverted = true;
        RobotConstants.frontRightInverted = false;
        RobotConstants.rearRightInverted = false;
        RobotConstants.rearLeftInverted = true;

        RobotConstants.DEFAULT_SPEED_PERC = 1.0;
        RobotConstants.SLOW_SPEED_PERC = 0.7;

        // ---------------------------- Transfer Pose from Autonomous --------------------------- //
//        pose = PoseStorage.currentPose;
    }

    public void initAllianceRelated(DecodeRobot.Alliance alliance) {
        if(alliance == DecodeRobot.Alliance.RED) {
//            pose = new Pose(-72 + 8.375, -8.5, 0);
            pose = new Pose(0.0, 0.0, 0.0);
        }
        robot = new DecodeRobot(
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
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }

    @Override
    public void reset() {
        super.reset();
        PoseStorage.currentPose = robot.getPose(); // In case we stop TeleOP midway
        robotMap.getLimelight().close();
    }
}