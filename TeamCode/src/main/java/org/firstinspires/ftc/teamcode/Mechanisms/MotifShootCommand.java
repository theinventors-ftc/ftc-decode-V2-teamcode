package org.firstinspires.ftc.teamcode.Mechanisms;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.MotifStorage;
import org.firstinspires.ftc.teamcode.Util.Timer;

public class MotifShootCommand extends CommandBase {
    private Passthough passthough;
    private Timer timer;
    private double finger_hold_ms, finger_between_ms;
    private MotifStorage.Motif motif;
    private int idx = 0;

    public MotifShootCommand(Passthough passthough, double finger_hold_ms, double finger_between_ms, MotifStorage.Motif motif) {
        this.passthough = passthough;
        this.finger_hold_ms = finger_hold_ms;
        this.finger_between_ms = finger_between_ms;
        this.motif = motif;
    }

    @Override
    public void initialize() {
        timer = new Timer();
    }

//    @Override
//    public void execute() {
//        switch (idx) {
//            case 0:
//                passthough.setState();
//        }
//    }
}
