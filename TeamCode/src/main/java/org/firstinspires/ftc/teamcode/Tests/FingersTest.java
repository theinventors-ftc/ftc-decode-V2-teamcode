package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Disabled
@Config
@TeleOp(name="FingersTest", group="Tests")
public class FingersTest extends LinearOpMode {
    private ServoImplEx fingerF, fingerC, fingerR;
    public static double posF = 0.5, posC = 0.5, posR = 0.5;

    @Override
    public void runOpMode() {
        fingerF = hardwareMap.get(ServoImplEx.class, "fingerF");
        fingerC = hardwareMap.get(ServoImplEx.class, "fingerC");
        fingerR = hardwareMap.get(ServoImplEx.class, "fingerR");
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            fingerF.setPosition(posF);
            fingerC.setPosition(posC);
            fingerR.setPosition(posR);
        }
    }}
