package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.Detection;
import org.firstinspires.ftc.teamcode.RobotMap;

//@Disabled
@Config
@TeleOp(name="MotifTest", group="Tests")
public class MotifTest extends CommandOpMode {
    private RobotMap robotMap;
    private Detection detection;

    @Override
    public void initialize() {
        robotMap = new RobotMap(hardwareMap, telemetry);
        detection = new Detection(robotMap);
    }

    @Override
    public void run() {
        super.run();
        telemetry.update();
    }
}
