package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    private DoubleSupplier forward, strafe, rotation;

    public DriveSubsystem(HardwareMap hardwareMap) {
        List<Motor> motors = Stream.of(
                        "leftFront", "rightFront",
                        "leftBack", "rightBack")
                .map(name -> new Motor(hardwareMap, name, Motor.GoBILDA.RPM_435))
                .collect(Collectors.toList());

        drive = new MecanumDrive(
                motors.get(3), motors.get(2),
                motors.get(1), motors.get(0)
        );

        motors.forEach(motor -> motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT));
    }

    public void setAxes(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
    }

    @Override
    public void periodic() {
        drive.driveRobotCentric(
                strafe.getAsDouble(),
                forward.getAsDouble(),
                -rotation.getAsDouble(),
                false
        );
    }

    public void setMaxSpeed(double speed) {
        drive.setMaxSpeed(speed);
    }
}
