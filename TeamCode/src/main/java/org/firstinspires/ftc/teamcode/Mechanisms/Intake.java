package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.RobotMap;

@Config
public class Intake extends SubsystemBase {
    private final MotorExEx intakeMotor;

    private final double INTAKE_POWER = 1.0;

    public enum IntakeState {
        INTAKE,
        REVERSE,
        STOPPED
    }

    private IntakeState state = IntakeState.STOPPED;
    private Telemetry telemetry;

    public Intake(RobotMap robotMap) {
        this.intakeMotor = robotMap.getIntakeMotor();
        intakeMotor.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setInverted(true);

        this.telemetry = robotMap.getTelemetry();
    }

    @Override
    public void periodic() {
        telemetry.addData("[Intake] State ", state);
    }

    public void intake() {
        state = IntakeState.INTAKE;
        intakeMotor.set(INTAKE_POWER);
    }

    public void reverse() {
        state = IntakeState.REVERSE;
        intakeMotor.set(-INTAKE_POWER);
    }

    public void stop() {
        state = IntakeState.STOPPED;
        intakeMotor.set(0);
    }

    public IntakeState getState() {
        return state;
    }
}
