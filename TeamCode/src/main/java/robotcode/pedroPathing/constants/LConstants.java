package robotcode.pedroPathing.constants;

import com.pedropathing.localization.constants.OTOSConstants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

public class LConstants {
    static {
        OTOSConstants.useCorrectedOTOSClass = true;
        OTOSConstants.hardwareMapName = "sensor_otos";

        OTOSConstants.linearUnit = DistanceUnit.INCH;
        OTOSConstants.angleUnit = AngleUnit.RADIANS;
        
        OTOSConstants.offset = new SparkFunOTOS.Pose2D(-6.8503937008, 0, 0);

        double[] linearScalars = {0.9449, 0.9758, 0.9441, 0.9222};
        double[] angularScalars = {0.9742, 0.9818, 0.9809};

        OTOSConstants.linearScalar = Arrays.stream(linearScalars).sum() / linearScalars.length;
        OTOSConstants.angularScalar = Arrays.stream(angularScalars).sum() / angularScalars.length;
    }
}




