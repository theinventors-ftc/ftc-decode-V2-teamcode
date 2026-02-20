package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="DriveMotorsTest", group="Tests")
//@Disabled
public class DriveMotorsTest extends LinearOpMode {
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeftDrive = hardwareMap.get(DcMotor.class, "rearLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        backRightDrive = hardwareMap.get(DcMotor.class, "rearRight");
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double frontLeftPower  = gamepad1.x ? 0.2 : 0.0;  // X gamepad
            double backLeftPower   = gamepad1.a ? 0.2 : 0.0;  // A gamepad
            double frontRightPower = gamepad1.y ? 0.2 : 0.0;  // Y gamepad
            double backRightPower  = gamepad1.b ? 0.2 : 0.0;  // B gamepad
//            x 0, a 2, y 1, b 3

            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);
        }
    }}
