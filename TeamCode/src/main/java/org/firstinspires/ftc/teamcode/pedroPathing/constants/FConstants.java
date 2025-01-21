package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.OTOS;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftBack";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightBack";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 15.25;

        FollowerConstants.xMovement = 70.88308259258119;
        FollowerConstants.yMovement = 56.402852216104826;

        FollowerConstants.forwardZeroPowerAcceleration = -29.333838928699272;
        FollowerConstants.lateralZeroPowerAcceleration = -78.19014779427714;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1, 0, 0.01, 0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.25, 0, 0.02, 0);

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2, 0, 0.1, 0);
        FollowerConstants.useSecondaryHeadingPID = false;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2, 0, 0.1, 0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01, 0, 0.0002, 1, 0);
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.1, 0, 0, 0.6, 0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 6;
        FollowerConstants.centripetalScaling = 0.0003;

        FollowerConstants.holdPointTranslationalScaling = 2.0 / 3.0;
        FollowerConstants.holdPointHeadingScaling = 0.5;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.9;
        FollowerConstants.pathEndVelocityConstraint = 100;
        FollowerConstants.pathEndTranslationalConstraint = 0.5;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
