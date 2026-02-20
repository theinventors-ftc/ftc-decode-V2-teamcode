package org.firstinspires.ftc.teamcode.PurePursuit.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;

@Disabled
@Config
@Autonomous(name = "PID Test", group = "Test")
public class PIDTest extends LinearOpMode {

    private RobotMap robotMap;
    private RobotMovement rm;

    private Pose startingPose = new Pose(0,0,0);
    private Pose goal = new Pose(20,20,180);

    @Override
    public void runOpMode() {
        robotMap = new RobotMap(hardwareMap, telemetry);
        rm = new RobotMovement(robotMap, startingPose);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {

            rm.updateLocalizer();
            Pose currentPose = rm.getCurrentPose();

            double realTranslationalEndDistance = Math.hypot(goal.getX() - currentPose.getX(),
                                                             goal.getY() - currentPose.getY());

            double realThetaEndDistance = goal.getTheta() - currentPose.getTheta();

            Pose motorPowers = rm.goToPoint(goal, currentPose, realTranslationalEndDistance,
                         realThetaEndDistance);

//            rm.robotCentricMovement(rm.turnToRobotCentric(motorPowers, currentPose)); // TODO: Ask George

            telemetry.addData("X: ", currentPose.getX());
            telemetry.addData("Y: ", currentPose.getY());
            telemetry.addData("Theta: ", currentPose.getTheta());
            telemetry.addData("Motor X: ", motorPowers.getX());
            telemetry.addData("Motor Y: ", motorPowers.getY());
            telemetry.addData("Motor Theta: ", motorPowers.getTheta());
            telemetry.update();
        }
    }
}