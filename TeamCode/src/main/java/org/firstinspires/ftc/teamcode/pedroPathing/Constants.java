package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
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
        .mass(13)

        .lateralZeroPowerAcceleration(-70.87594153)
        .forwardZeroPowerAcceleration(-30.73492250)

        .useSecondaryTranslationalPIDF(true)
        .useSecondaryHeadingPIDF(true)
        .useSecondaryDrivePIDF(false)

        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0.0, 0.0, 0.6, 0.0))
        .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02,0,0.001,0.6,0.0))

        .translationalPIDFCoefficients(new PIDFCoefficients(0.23, 0.00005, 0.02, 0))
        .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.27,0.00007,0.028,0))

        .headingPIDFCoefficients(new PIDFCoefficients(1.8, 0, 0.002, 0.008))
        .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2,0.01,0.2,0));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("rearRight")
            .leftRearMotorName("rearLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(93.64337686)
            .yVelocity(69.47181880);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(7.6377952755905511811023622047244)
            .strafePodX(-0.001)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odometry")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
