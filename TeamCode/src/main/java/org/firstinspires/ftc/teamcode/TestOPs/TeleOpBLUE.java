package org.firstinspires.ftc.teamcode.TestOPs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DecodeRobot;

@TeleOp(name = "TeleOP_BLUE", group = "")
public class TeleOpBLUE extends TeleOpBase {
    @Override
    public void initialize() {
        super.initialize();
        this.initAllianceRelated(DecodeRobot.Alliance.BLUE);
    }
}