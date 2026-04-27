package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
            .mass(13.75)   // mass in kg
            .forwardZeroPowerAcceleration(-41.11714300428105)   // have to be negative
            .lateralZeroPowerAcceleration(-72.05154497545118)
            //  .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.015, 0.04))    // these are the PIDF
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.07, 0.03))
            //  .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05,0.0,0.00001,0.6,0.05))
            .centripetalScaling(0.0)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.1,0.0938469031342008,0.0010586740774408687))
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)   // max motor power
            .rightFrontMotorName("rf")   // Motor hardware maps
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)  // Motor directions
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(71.6025885634535)
            .yVelocity(53.65914124015749)
            .useBrakeModeInTeleOp(true);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(4.09)   // Y offset
            .strafePodX(-6.6)  // X offset
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")  // hardware map for pinpoint computer
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)   // Encoder direction
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
