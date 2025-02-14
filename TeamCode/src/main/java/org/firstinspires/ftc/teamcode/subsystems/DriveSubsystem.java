package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

import java.util.function.DoubleSupplier;

@Config
public class DriveSubsystem extends SubsystemBase {
    public static PIDCoefficients
            headingPIDCoefficients = new PIDCoefficients(0, 0, 0),
            forwardPIDCoefficients = new PIDCoefficients(0, 0, 0),
            strafePIDCoefficients = new PIDCoefficients(0, 0, 0);

    public static double TRANSLATIONAL_CONSTRAINT = 0.1;
    public static double HEADING_CONSTRAINT = 0.4;

    private final PIDController headingPID, forwardPID, strafePID;
    private final Motor leftFront, rightFront, rightBack, leftBack;
    private final GoBildaPinpointDriver pinpoint;
    private double maxSpeed = 1;
    private boolean teleOpMode = false;
    private boolean enableTrackingInTeleOp = true;
    private DoubleSupplier forward, strafe, rotation;
    private Pose2D targetPosition;

    public DriveSubsystem(HardwareMap hardwareMap) {
        leftFront = new Motor(hardwareMap, "leftFront", Motor.GoBILDA.RPM_435);
        rightFront = new Motor(hardwareMap, "rightFront", Motor.GoBILDA.RPM_435);
        leftBack = new Motor(hardwareMap, "leftBack", Motor.GoBILDA.RPM_435);
        rightBack = new Motor(hardwareMap, "rightBack", Motor.GoBILDA.RPM_435);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        pinpoint.setOffsets(-117.5, 65);

        pinpoint.resetPosAndIMU();

        leftFront.setInverted(true);
        leftBack.setInverted(true);

        headingPID = new PIDController(
                headingPIDCoefficients.p,
                headingPIDCoefficients.i,
                headingPIDCoefficients.d
        );
        forwardPID = new PIDController(
                forwardPIDCoefficients.p,
                forwardPIDCoefficients.i,
                forwardPIDCoefficients.d
        );
        strafePID = new PIDController(
                strafePIDCoefficients.p,
                strafePIDCoefficients.i,
                strafePIDCoefficients.d
        );
    }


    @Override
    public void periodic() {
        if (!teleOpMode || enableTrackingInTeleOp) {
            pinpoint.update();
        }

        headingPID.setPID(
                headingPIDCoefficients.p,
                headingPIDCoefficients.i,
                headingPIDCoefficients.d
        );
        forwardPID.setPID(
                forwardPIDCoefficients.p,
                forwardPIDCoefficients.i,
                forwardPIDCoefficients.d
        );
        strafePID.setPID(
                strafePIDCoefficients.p,
                strafePIDCoefficients.i,
                strafePIDCoefficients.d
        );

        if (teleOpMode) {
            double forwardSpeed = -forward.getAsDouble();
            double strafeSpeed = strafe.getAsDouble() * 1.1;
            double rotationSpeed = rotation.getAsDouble();

            double denominator = Math.max(Math.abs(forwardSpeed) + Math.abs(strafeSpeed) + Math.abs(rotationSpeed), 1);
            leftFront.set((forwardSpeed + strafeSpeed + rotationSpeed) / denominator * maxSpeed);
            leftBack.set((forwardSpeed - strafeSpeed + rotationSpeed) / denominator * maxSpeed);
            rightFront.set((forwardSpeed - strafeSpeed - rotationSpeed) / denominator * maxSpeed);
            rightBack.set((forwardSpeed + strafeSpeed - rotationSpeed) / denominator * maxSpeed);
        } else if (targetPosition != null) {
            Pose2D currentPosition = getPosition();

            Vector2d targetVector = new Vector2d(
                    forwardPID.calculate(currentPosition.getX(DistanceUnit.INCH), targetPosition.getX(DistanceUnit.INCH)),
                    strafePID.calculate(currentPosition.getY(DistanceUnit.INCH), targetPosition.getY(DistanceUnit.INCH))
            );
            targetVector = targetVector.rotateBy(
                    AngleUnit.normalizeDegrees(currentPosition.getHeading(AngleUnit.DEGREES))
            );

            double forwardSpeed = targetVector.getX();
            double strafeSpeed = targetVector.getY();
            double rotationSpeed = headingPID.calculate(
                    AngleUnit.normalizeRadians(currentPosition.getHeading(AngleUnit.RADIANS)),
                    AngleUnit.normalizeRadians(targetPosition.getHeading(AngleUnit.RADIANS))
            );

            leftFront.set((forwardSpeed + strafeSpeed + rotationSpeed) * maxSpeed);
            leftBack.set((forwardSpeed - strafeSpeed + rotationSpeed) * maxSpeed);
            rightFront.set((forwardSpeed - strafeSpeed - rotationSpeed) * maxSpeed);
            rightBack.set((forwardSpeed + strafeSpeed - rotationSpeed) * maxSpeed);
        } else {
            leftFront.set(0);
            leftBack.set(0);
            rightFront.set(0);
            rightBack.set(0);
        }
    }

    public void setMotorBehavior(Motor.ZeroPowerBehavior behavior) {
        leftFront.setZeroPowerBehavior(behavior);
        leftBack.setZeroPowerBehavior(behavior);
        rightFront.setZeroPowerBehavior(behavior);
        rightBack.setZeroPowerBehavior(behavior);
    }

    public void setTeleOpMode(boolean mode, boolean enableTracking) {
        teleOpMode = mode;
        enableTrackingInTeleOp = enableTracking;
    }

    public void setTeleOpMode(boolean mode) {
        setTeleOpMode(mode, false);
    }

    public void setTeleOpAxes(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
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

    public void setPosition(double x, double y, double h) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, AngleUnit.normalizeDegrees(h)));
    }

    public void setPosition(Pose2D pose) {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                AngleUnit.DEGREES,
                AngleUnit.normalizeDegrees(pose.getHeading(AngleUnit.DEGREES))
        ));
    }

    public Pose2D getPosition() {
        if (Double.isNaN(pinpoint.getPosition().getX(DistanceUnit.INCH)) || Double.isNaN(pinpoint.getPosition().getY(DistanceUnit.INCH)) || Double.isNaN(pinpoint.getPosition().getHeading(AngleUnit.RADIANS))) {
            if (targetPosition != null) {
                return targetPosition;
            } else {
                return new Pose2D(
                        DistanceUnit.INCH,
                        0,
                        0,
                        AngleUnit.DEGREES,
                        AngleUnit.normalizeDegrees(0)
                );
            }
        } else {
            return new Pose2D(
                    DistanceUnit.MM,
                    pinpoint.getPosX(),
                    pinpoint.getPosY(),
                    AngleUnit.RADIANS,
                    AngleUnit.normalizeRadians(pinpoint.getHeading())
            );
        }
    }

    public void goToPosition(double x, double y, double h) {
        targetPosition = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, AngleUnit.normalizeDegrees(h));
    }

    public void goToPosition(@NonNull Pose2D pose) {
        targetPosition = new Pose2D(
                DistanceUnit.INCH,
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                AngleUnit.DEGREES,
                AngleUnit.normalizeDegrees(pose.getHeading(AngleUnit.DEGREES))
        );
    }

    public boolean atTarget() {
        return getDistanceError(DistanceUnit.INCH) < TRANSLATIONAL_CONSTRAINT && getHeadingError(AngleUnit.DEGREES) < HEADING_CONSTRAINT;
    }

    public double getXError(DistanceUnit unit) {
        if (targetPosition == null) {
            return 0;
        }

        return Math.abs(targetPosition.getX(unit) - getPosition().getX(unit));
    }

    public double getYError(DistanceUnit unit) {
        if (targetPosition == null) {
            return 0;
        }

        return Math.abs(targetPosition.getY(unit) - getPosition().getY(unit));
    }

    public double getHeadingError(AngleUnit unit) {
        if (targetPosition == null) {
            return 0;
        }

        return Math.abs(AngleUnit.normalizeDegrees(targetPosition.getHeading(unit)) - AngleUnit.normalizeDegrees(getPosition().getHeading(unit)));
    }

    public double getDistanceError(DistanceUnit unit) {
        if (targetPosition == null) {
            return 0;
        }

        return Math.sqrt(
                Math.pow(targetPosition.getX(unit) - getPosition().getX(unit), 2) + Math.pow(targetPosition.getY(unit) - getPosition().getY(unit), 2)
        );
    }

    public void removeTarget() {
        targetPosition = null;
    }

    public Pose2D getTarget() {
        return targetPosition;
    }
}
