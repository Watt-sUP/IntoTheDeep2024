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
        TwoWheelOTOSConstants.offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        TwoWheelOTOSConstants.angularScalar = .994;

        TwoWheelOTOSConstants.forwardEncoder_HardwareMapName = "leftBack";
        TwoWheelOTOSConstants.strafeEncoder_HardwareMapName = "leftFront";

        TwoWheelOTOSConstants.forwardEncoderDirection = Encoder.FORWARD;
        TwoWheelOTOSConstants.strafeEncoderDirection = Encoder.FORWARD;

        TwoWheelOTOSConstants.forwardTicksToInches = 0.0019877459810144797;
        TwoWheelOTOSConstants.strafeTicksToInches = 0.0019944979727697216;

        // 4.8622047244
        TwoWheelOTOSConstants.forwardY = 2637.7;
        // 2.6377952756
        TwoWheelOTOSConstants.strafeX = 2637.7;
    }
}
