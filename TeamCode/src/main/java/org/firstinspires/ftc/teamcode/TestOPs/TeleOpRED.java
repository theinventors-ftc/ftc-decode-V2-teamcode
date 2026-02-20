package org.firstinspires.ftc.teamcode.TestOPs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DecodeRobotV2;

@TeleOp(name = "TeleOP_RED", group = "")
public class TeleOpRED extends TeleOpBase {
    @Override
    public void initialize() {
        super.initialize();
        this.initAllianceRelated(DecodeRobotV2.Alliance.RED);
    }
}