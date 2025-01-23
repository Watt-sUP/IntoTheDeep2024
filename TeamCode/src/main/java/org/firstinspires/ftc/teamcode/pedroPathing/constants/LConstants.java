package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.constants.OTOSConstants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {

    static {
        OTOSConstants.useCorrectedOTOSClass = false;
        OTOSConstants.hardwareMapName = "sensor_otos";

        OTOSConstants.linearUnit = DistanceUnit.INCH;
        OTOSConstants.angleUnit = AngleUnit.RADIANS;

        OTOSConstants.offset = new SparkFunOTOS.Pose2D(-6.8897637795275, 0, 0);
        OTOSConstants.useAccelerometer = false;

        OTOSConstants.linearScalar = .97433;
        OTOSConstants.angularScalar = .994;
    }
}
