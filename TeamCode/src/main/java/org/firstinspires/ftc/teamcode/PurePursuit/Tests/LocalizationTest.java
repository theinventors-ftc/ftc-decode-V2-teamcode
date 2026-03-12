package org.firstinspires.ftc.teamcode.PurePursuit.Tests;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.Localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.RobotMap;


@Disabled
@TeleOp(name = "Localization Test", group = "Test")
public class LocalizationTest extends LinearOpMode {

    private RobotMap robotMap;
    private PinpointLocalizer localizer;

    private Pose startingPose = new Pose(0,0,0);

    /*-- Movement --*/
    public void robotCentricMovement(Pose pose) {
        // Parallel encoder motion Y Axis
        // Perpendicular encoder motion X Axis
        // Rotational IMU motion Theta

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(pose.getY()) + abs(pose.getX()) + abs(pose.getTheta()), 1);
        double frontLeftPower = (pose.getY() + pose.getX() + pose.getTheta()) / denominator;
        double backLeftPower = (pose.getY() - pose.getX() + pose.getTheta()) / denominator;
        double frontRightPower = (pose.getY() - pose.getX() - pose.getTheta()) / denominator;
        double backRightPower = (pose.getY() + pose.getX() - pose.getTheta()) / denominator;

        robotMap.getFrontLeftMotor().set(frontLeftPower);
        robotMap.getRearLeftMotor().set(backLeftPower);
        robotMap.getFrontRightMotor().set(frontRightPower);
        robotMap.getRearRightMotor().set(backRightPower);
    }

    @Override
    public void runOpMode() {
        robotMap = new RobotMap(hardwareMap, telemetry);
        localizer = new PinpointLocalizer(robotMap, startingPose);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            robotCentricMovement(new Pose(gamepad1.left_stick_x,
                                          -gamepad1.left_stick_y,
                                          gamepad1.right_stick_x
            ));
            localizer.update();

            telemetry.addData("X: ", localizer.getPose().getX());
            telemetry.addData("Y: ", localizer.getPose().getY());
            telemetry.addData("Theta: ", MathFunction.angleWrap(localizer.getPose().getTheta()));
            telemetry.update();

            FtcDashboard.getInstance().getTelemetry().update();
        }
    }
}
