package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

@Config
public class DriveSubsystem extends SubsystemBase {
    private final Motor leftFront, rightFront, rightBack, leftBack;
    private double maxSpeed = 1;
    private DoubleSupplier forward, strafe, rotation;

    public DriveSubsystem(HardwareMap hardwareMap) {
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);

        leftFront.setInverted(true);
        leftBack.setInverted(true);
    }

    public void setMotorBrake(boolean brake) {
        leftFront.setZeroPowerBehavior(brake ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(brake ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(brake ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(brake ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void periodic() {
        double forwardSpeed = forward.getAsDouble();
        double strafeSpeed = strafe.getAsDouble() * 1.1;
        double rotationSpeed = rotation.getAsDouble();

        double denominator = Math.max(Math.abs(forwardSpeed) + Math.abs(strafeSpeed) + Math.abs(rotationSpeed), 1);
        leftFront.set((forwardSpeed + strafeSpeed + rotationSpeed) / denominator * maxSpeed);
        leftBack.set((forwardSpeed - strafeSpeed + rotationSpeed) / denominator * maxSpeed);
        rightFront.set((forwardSpeed - strafeSpeed - rotationSpeed) / denominator * maxSpeed);
        rightBack.set((forwardSpeed + strafeSpeed - rotationSpeed) / denominator * maxSpeed);
    }

    public void setAxes(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
    }

    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }
}
