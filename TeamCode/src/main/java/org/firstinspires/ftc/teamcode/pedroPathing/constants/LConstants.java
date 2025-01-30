package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.TwoWheelOTOSConstants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class LConstants {
//    static {
//        OTOSConstants.useCorrectedOTOSClass = false;
//        OTOSConstants.hardwareMapName = "sensor_otos";
//
//        OTOSConstants.linearUnit = DistanceUnit.INCH;
//        OTOSConstants.angleUnit = AngleUnit.RADIANS;
//
//        OTOSConstants.offset = new SparkFunOTOS.Pose2D(-6.8897637795275, 0, 0);
//
//        OTOSConstants.linearScalar = .99;
//        OTOSConstants.angularScalar = .994;
//    }

    static {
        TwoWheelOTOSConstants.OTOS_HardwareMapName = "sensor_otos";
        TwoWheelOTOSConstants.offset = new SparkFunOTOS.Pose2D(-6.8897637795275, 0, 0);
        TwoWheelOTOSConstants.angularScalar = .994;

        TwoWheelOTOSConstants.forwardEncoder_HardwareMapName = "leftFront";
        TwoWheelOTOSConstants.strafeEncoder_HardwareMapName = "leftBack";

        TwoWheelOTOSConstants.forwardEncoderDirection = Encoder.FORWARD;
        TwoWheelOTOSConstants.strafeEncoderDirection = Encoder.FORWARD;

        TwoWheelOTOSConstants.forwardTicksToInches = .001989436789;
        TwoWheelOTOSConstants.strafeTicksToInches = .001989436789;

        TwoWheelOTOSConstants.forwardY = 1;
        TwoWheelOTOSConstants.strafeX = -2.5;
    }
}
