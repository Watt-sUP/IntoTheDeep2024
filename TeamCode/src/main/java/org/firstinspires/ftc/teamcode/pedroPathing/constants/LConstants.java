package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.DriveEncoderConstants;

public class LConstants {

    static {
        DriveEncoderConstants.forwardTicksToInches = 0.0082434;
        DriveEncoderConstants.strafeTicksToInches = 0.0089245471118;
        DriveEncoderConstants.turnTicksToInches = 0.0198359617;

        DriveEncoderConstants.robot_Width = 11.25;
        DriveEncoderConstants.robot_Length = 14.50;

        DriveEncoderConstants.leftFrontEncoderDirection = Encoder.REVERSE;
        DriveEncoderConstants.leftRearEncoderDirection = Encoder.REVERSE;
        DriveEncoderConstants.rightFrontEncoderDirection = Encoder.FORWARD;
        DriveEncoderConstants.rightRearEncoderDirection = Encoder.FORWARD;
    }
}
