package robotcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;
import java.util.stream.Collectors;

import robotcode.util.IndependentMotorGroup;

public class AscentSubsystem extends SubsystemBase {
    public static double kP = 0.5;
    private final IndependentMotorGroup elevator;

    private AscentState elevatorState = AscentState.DOWN;

    public AscentSubsystem(HardwareMap hardwareMap) {
        elevator = new IndependentMotorGroup(
                new Motor(hardwareMap, "ascent_left"),
                new Motor(hardwareMap, "ascent_right")
        );

        elevator.setGroupType(384.5, 435);
        elevator.setInverted(true);

        elevator.stopAndResetEncoder();
        elevator.setDistancePerPulse(getDegreesPerTick());
        elevator.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        elevator.setPositionTolerance(1.0);
        elevator.setPositionCoefficient(kP);
        elevator.setRunMode(Motor.RunMode.PositionControl);
    }

    @Override
    public void periodic() {
        if (elevator.atTargetPosition()) {
            elevator.stopMotors();
            return;
        }

        elevator.set(1.0);
    }

    private double getDegreesPerTick() {
        assert Double.isFinite(elevator.getCPR()) : "The CPR of the elevator is undefined";

        final double GEAR_RATIO = 28.0;
        return 360.0 / (elevator.getCPR() * GEAR_RATIO);
    }

    public void setElevatorAngle(double angle) {
        elevator.setTargetDistance(angle);
    }

    public void setElevatorState(AscentState state) {
        elevatorState = state;
        setElevatorAngle(elevatorState.getAngle());
    }

    public String getElevatorAngle() {
        return elevator.getDistances().stream()
                .map(distance -> String.format(Locale.ROOT, "%.2f", distance))
                .collect(Collectors.joining(", "));
    }

    public String getElevatorState() {
        return elevatorState.toString();
    }

    public enum AscentState {
        HOOKING(92), HANGING(45), DOWN(0);

        private final double angle;

        AscentState(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            return angle;
        }
    }
}
