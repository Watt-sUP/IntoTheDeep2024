package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class SampleSensor {
    public static double THRESHOLD = 20;
    public static double MIN_DISTANCE = 1;
    public static double MAX_DISTANCE = 3;

    public static LABColor RED_MATCH = new LABColor(48, 39, 44);
    public static LABColor BLUE_MATCH = new LABColor(39, 39, -76);
    public static LABColor YELLOW_MATCH = new LABColor(97, -19, 76);

    public enum SampleType {
        Blue,
        Red,
        Yellow,
        None;

        @NonNull
        @Override
        public String toString() {
            switch (this) {
                case Blue:
                    return "Blue";
                case Red:
                    return "Red";
                case Yellow:
                    return "Yellow";
                default:
                    return "None";
            }
        }
    }

    private static RevColorSensorV3 sensor;

    public SampleSensor(@NonNull HardwareMap hardwareMap) {
        sensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
    }

    public double deltaRed;
    public double deltaBlue;
    public double deltaYellow;

    public SampleType getSampleType() {
        NormalizedRGBA colors = sensor.getNormalizedColors();

        LABColor color = new LABColor(colors);
        double distance = sensor.getDistance(DistanceUnit.CM);

        deltaRed = RED_MATCH.deltaE(color);
        deltaBlue = BLUE_MATCH.deltaE(color);
        deltaYellow = YELLOW_MATCH.deltaE(color);

        if (distance <= MAX_DISTANCE && distance >= MIN_DISTANCE) {
            if ((deltaRed < deltaBlue && deltaRed < deltaYellow) && deltaRed <= THRESHOLD) {
                return SampleType.Red;
            } else if ((deltaBlue < deltaRed && deltaBlue < deltaYellow) && deltaBlue <= THRESHOLD) {
                return SampleType.Blue;
            } else if ((deltaYellow < deltaRed && deltaYellow < deltaBlue) && deltaYellow <= THRESHOLD) {
                return SampleType.Yellow;
            }
        }

        return SampleType.None;
    }

    public LABColor getColor() {
        NormalizedRGBA colors = sensor.getNormalizedColors();
        return new LABColor(colors);
    }

    public double getDistance(DistanceUnit unit) {
        return sensor.getDistance(unit);
    }
}

