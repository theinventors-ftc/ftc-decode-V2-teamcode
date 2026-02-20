package org.firstinspires.ftc.teamcode.Mechanisms;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.DecodeRobot;

public class CommandSeriesVault {
    private DecodeRobot.Alliance alliance;
    private Intake intake;
    private Passthough passthough;
    private Shooter shooter;

    // --------------------------------------- Constants ---------------------------------------- //
    public static int FINGER_BETWEEN_MS = 80, FINGER_HOLD_MS = 340;

    public CommandSeriesVault(Intake intake, Passthough passthough, Shooter shooter) {
        this.intake = intake;
        this.passthough = passthough;
        this.shooter = shooter;
    }

    public SequentialCommandGroup feedOneFinger(int fingerIdx) {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> shooter.wheelsAtSpeed()),
                new InstantCommand(() -> passthough.setState(fingerIdx, Passthough.FingerState.FEED), passthough),
                new WaitCommand(FINGER_HOLD_MS),
                new InstantCommand(() -> passthough.setState(fingerIdx, Passthough.FingerState.HOLD), passthough)
        );
    }

    public SequentialCommandGroup feedAllFingers() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> shooter.wheelsAtSpeed()),
                new InstantCommand(() -> passthough.setState(0, Passthough.FingerState.FEED), passthough),
                new WaitCommand(FINGER_HOLD_MS),
                new InstantCommand(() -> passthough.setState(0, Passthough.FingerState.HOLD), passthough),
                new WaitCommand(FINGER_BETWEEN_MS),
                new InstantCommand(() -> passthough.setState(1, Passthough.FingerState.FEED), passthough),
                new WaitCommand(FINGER_HOLD_MS),
                new InstantCommand(() -> passthough.setState(1, Passthough.FingerState.HOLD), passthough),
                new WaitCommand(FINGER_BETWEEN_MS),
                new InstantCommand(() -> passthough.setState(2, Passthough.FingerState.FEED), passthough),
                new WaitCommand(FINGER_HOLD_MS),
                new InstantCommand(() -> passthough.setState(2, Passthough.FingerState.HOLD), passthough),
                new WaitCommand(FINGER_BETWEEN_MS)
        );
    }

    public SequentialCommandGroup rearrangeArtifacts() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> passthough.setState(0, Passthough.FingerState.REARRANGE), passthough),
                new InstantCommand(() -> passthough.setState(1, Passthough.FingerState.REARRANGE), passthough),
                new InstantCommand(() -> passthough.setState(2, Passthough.FingerState.REARRANGE), passthough),
                new WaitCommand(150),
                new InstantCommand(() -> passthough.setState(0, Passthough.FingerState.HOLD), passthough),
                new InstantCommand(() -> passthough.setState(1, Passthough.FingerState.HOLD), passthough),
                new InstantCommand(() -> passthough.setState(2, Passthough.FingerState.HOLD), passthough),
                new WaitCommand(150),
                new InstantCommand(() -> passthough.setState(0, Passthough.FingerState.REARRANGE), passthough),
                new InstantCommand(() -> passthough.setState(1, Passthough.FingerState.REARRANGE), passthough),
                new InstantCommand(() -> passthough.setState(2, Passthough.FingerState.REARRANGE), passthough),
                new WaitCommand(150),
                new InstantCommand(() -> passthough.setState(0, Passthough.FingerState.HOLD), passthough),
                new InstantCommand(() -> passthough.setState(1, Passthough.FingerState.HOLD), passthough),
                new InstantCommand(() -> passthough.setState(2, Passthough.FingerState.HOLD), passthough)
        );
    }

    public InstantCommand enableWheels() {
        return new InstantCommand(() -> shooter.enableWheels(), shooter);
    }

    public InstantCommand disableWheels() {
        return new InstantCommand(() -> shooter.disableWheels(), shooter);
    }

    public InstantCommand reverseIntake() {
        return new InstantCommand(intake::reverse, intake);
    }

    public InstantCommand stopIntake() {
        return new InstantCommand(intake::stop, intake);
    }

    public SequentialCommandGroup startIntakeProc() {
        return new SequentialCommandGroup(
                new InstantCommand(intake::intake, intake),
                new InstantCommand(() -> passthough.setState(0, Passthough.FingerState.INTAKE), passthough),
                new InstantCommand(() -> passthough.setState(1, Passthough.FingerState.INTAKE), passthough),
                new InstantCommand(() -> passthough.setState(2, Passthough.FingerState.INTAKE), passthough)
        );
    }

    public SequentialCommandGroup stopIntakeProc() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> passthough.setState(0, Passthough.FingerState.HOLD), passthough),
                new InstantCommand(() -> passthough.setState(1, Passthough.FingerState.HOLD), passthough),
                new InstantCommand(() -> passthough.setState(2, Passthough.FingerState.HOLD), passthough),
                new InstantCommand(intake::stop, intake)
        );
    }

    public SequentialCommandGroup autonomousWaitForTurret() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> shooter.turretAtGoal()),
                new WaitUntilCommand(() -> shooter.wheelsAtSpeed())
        );
    }
}
