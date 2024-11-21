package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.InterpolatedAngleServo;
import org.firstinspires.ftc.teamcode.util.InterpolatedPositionServo;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double MOTOR_POWER = 0.75;

    public static double EXTENDO_IN = 0.12, EXTENDO_OUT = 1;
    public static double PIVOT_DOWN = 85, PIVOT_UP = 63;

    private final InterpolatedPositionServo extLeft, extRight;
    private final InterpolatedAngleServo pivLeft, pivRight;

//    private final MotorEx motor;
//    private final SampleSensor sensor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
//        motor = new MotorEx(hardwareMap, "active_motor", Motor.GoBILDA.RPM_1150);
//        motor.setRunMode(Motor.RunMode.RawPower);

        extLeft = new InterpolatedPositionServo(new SimpleServo(hardwareMap, "ext_left", 0, 220));
        extRight = new InterpolatedPositionServo(new SimpleServo(hardwareMap, "ext_right", 0, 220));

        extLeft.setInverted(false);
        extRight.setInverted(true);

        extLeft.generatePositions(
                new Pair<>(0.0, 0.62),
                new Pair<>(0.5, 0.76),
                new Pair<>(1.0, 0.96)
        );

        extRight.generatePositions(
                new Pair<>(0.0, 0.52),
                new Pair<>(0.5, 0.67),
                new Pair<>(1.0, 0.87)
        );

        pivLeft = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "int_left", 0, 220));
        pivRight = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "int_right", 0, 220));

        pivLeft.setInverted(false);
        pivRight.setInverted(true);

        pivLeft.generatePositions(
                new Pair<>(0.0, 28.0),
                new Pair<>(90.0, 125.0),
                new Pair<>(180.0, 218.0)
        );

        pivRight.generatePositions(
                new Pair<>(0.0, 20.0),
                new Pair<>(90.0, 114.0),
                new Pair<>(180.0, 211.0)
        );

//        sensor = new SampleSensor(hardwareMap);
    }

    public void extend() {
        extLeft.setToPosition(EXTENDO_OUT);
        extRight.setToPosition(EXTENDO_OUT);

        pivLeft.setToPosition(PIVOT_DOWN);
        pivRight.setToPosition(PIVOT_DOWN);
    }

    public void retract() {
        extLeft.setToPosition(EXTENDO_IN);
        extRight.setToPosition(EXTENDO_IN);

        pivLeft.setToPosition(PIVOT_UP);
        pivRight.setToPosition(PIVOT_UP);
    }
//
//    public void collect() {
//        motor.set(MOTOR_POWER);
//    }
//
//    public void remove() {
//        motor.set(-MOTOR_POWER);
//    }
//
//    public void stop() {
//        motor.set(0);
//    }

//    public SampleSensor.SampleType getSampleDetected() {
//        return sensor.getSampleType();
//    }
//
//    public LABColor getColorDetected() {
//        return sensor.getColor();
//    }
//
//    public double getSensorDistance(DistanceUnit unit) {
//        return sensor.getDistance(unit);
//    }
}
