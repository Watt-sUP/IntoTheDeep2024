package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.InterpolatedAngleServo;
import org.firstinspires.ftc.teamcode.util.InterpolatedPositionServo;
import org.firstinspires.ftc.teamcode.util.LABColor;
import org.firstinspires.ftc.teamcode.util.SampleSensor;

import java.util.concurrent.TimeUnit;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double MOTOR_POWER = 0.25;

    private final InterpolatedPositionServo extLeft, extRight;
    private final InterpolatedAngleServo pivLeft, pivRight;

    private final MotorEx motor;
    private final SampleSensor sensor;

    private boolean isCollecting = false;
    private boolean isRemoving = false;
    private boolean hadBadSample = false;

    private final Timing.Timer timer = new Timing.Timer(250, TimeUnit.MILLISECONDS);

    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = new MotorEx(hardwareMap, "active_motor", Motor.GoBILDA.RPM_1150);
        motor.setRunMode(Motor.RunMode.RawPower);

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

        sensor = new SampleSensor(hardwareMap);

        timer.pause();
    }

    public void startCollecting() {
        isCollecting = true;
        isRemoving = false;
    }

    public void stopCollecting() {
        isCollecting = false;
    }

    public boolean isCollecting() {
        return isCollecting;
    }

    public void startRemoving() {
        isRemoving = true;
        isCollecting = false;
    }

    public void stopRemoving() {
        isRemoving = false;
    }

    public SampleSensor.SampleType getSampleDetected() {
        return sensor.getSampleType();
    }

    public LABColor getColorDetected() {
        return sensor.getColor();
    }

    public double getSensorDistance(DistanceUnit unit) {
        return sensor.getDistance(unit);
    }

    @Override
    public void periodic() {
        if (isCollecting || isRemoving) {
            extLeft.setToPosition(1);
            extRight.setToPosition(1);
        }

        if (isCollecting) {

        }

        extLeft.setToPosition(0);
        extRight.setToPosition(0);
        motor.set(0);
    }
}
