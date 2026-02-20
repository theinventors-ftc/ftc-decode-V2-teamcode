package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9)
            .lateralZeroPowerAcceleration(-33.60755547)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.11, 0.00005, 0.012, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.25,0.00006,0.075,0))
            .headingPIDFCoefficients(new PIDFCoefficients(1.8, 0, 0.002, 0.008))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2,0.01,0.2,0));

//    public static FollowerConstants followerConstants = new FollowerConstants()
//            .mass(9)
//            .lateralZeroPowerAcceleration(-33.60755547)
//            .useSecondaryTranslationalPIDF(true)
//            .useSecondaryHeadingPIDF(true)
//            .useSecondaryDrivePIDF(true)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.11, 0.00005, 0.007, 0))
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.25,0.00006,0.017,0))
//            .headingPIDFCoefficients(new PIDFCoefficients(1.8, 0, 0.002, 0.008))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2,0.01,0.2,0));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("rearRight")
            .leftRearMotorName("rearLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(77.640039)
            .yVelocity(63.08406655);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.2362204724409)
            .strafePodX(-4.7244094488189)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odometry")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
