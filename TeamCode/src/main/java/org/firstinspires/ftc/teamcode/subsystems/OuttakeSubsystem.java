package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.InterpolatedAngleServo;

@Config
public class OuttakeSubsystem extends SubsystemBase {
    public enum ClawState {
        OPENED,
        CLOSED;

        @NonNull
        public String toString() {
            switch (this) {
                case OPENED:
                    return "Opened";
                case CLOSED:
                    return "Closed";
            }
            return "";
        }
    }

    public enum ArmState {
        IN,
        OUT;

        @NonNull
        public String toString() {
            switch (this) {
                case IN:
                    return "In";
                case OUT:
                    return "Out";
            }
            return "";
        }
    }

    public enum PivotState {
        IN,
        OUT;

        @NonNull
        public String toString() {
            switch (this) {
                case IN:
                    return "In";
                case OUT:
                    return "Out";
            }
            return "";
        }
    }

    public enum SlidesState {
        LOWERED,
        SPECIMEN,
        LOW_BASKET,
        HIGH_BASKET;

        @NonNull
        public String toString() {
            switch (this) {
                case LOWERED:
                    return "Lowered";
                case SPECIMEN:
                    return "Specimen";
                case LOW_BASKET:
                    return "Low Basket";
                case HIGH_BASKET:
                    return "High Basket";
            }
            return "";
        }

        public SlidesState next() {
            switch (this) {
                case LOWERED:
                    return SPECIMEN;
                case SPECIMEN:
                    return LOW_BASKET;
                case LOW_BASKET:
                case HIGH_BASKET:
                    return HIGH_BASKET;
            }
            return LOWERED;
        }

        public SlidesState previous() {
            switch (this) {
                case LOWERED:
                case SPECIMEN:
                    return LOWERED;
                case LOW_BASKET:
                    return SPECIMEN;
                case HIGH_BASKET:
                    return LOW_BASKET;
            }
            return LOWERED;
        }
    }

    public static double CLAW_CLOSED = 0, CLAW_OPEN = 0.55;
    public static double ARM_IN = 2, ARM_OUT = 220;
    public static double PIVOT_IN = 0.80, PIVOT_OUT = 0.4;

    public static double SLIDES_kP = 0;
    public static int SLIDES_LOWERED = 0,
            SLIDES_SPECIMEN = 10,
            SLIDES_LOW_BASKET = 20,
            SLIDES_HIGH_BASKET = 30;

    private final InterpolatedAngleServo armLeft;
    private final InterpolatedAngleServo armRight;
    private final SimpleServo armPivot;

    private final SimpleServo clawServo;
    private final Motor slidesMotor;

    private ClawState clawState = ClawState.CLOSED;
    private ArmState armState = ArmState.IN;
    private PivotState pivotState = PivotState.IN;
    private SlidesState slidesState = SlidesState.LOWERED;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        clawServo = new SimpleServo(hardwareMap, "claw_servo", 0, 360);
        clawServo.setInverted(false);

        armLeft = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "arm_left", 0, 220));
        armRight = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "arm_right", 0, 220));

        armLeft.setInverted(true);
        armRight.setInverted(false);

        armLeft.generatePositions(
                new Pair<>(0.0, 0.0),
                new Pair<>(90.0, 90.0),
                new Pair<>(180.0, 180.0),
                new Pair<>(220.0, 220.0)
        );

        armRight.generatePositions(
                new Pair<>(0.0, 14.0),
                new Pair<>(90.0, 91.0),
                new Pair<>(180.0, 175.0),
                new Pair<>(220.0, 197.0)
        );

        armPivot = new SimpleServo(hardwareMap, "arm_pivot", 0, 180);

        slidesMotor = new Motor(hardwareMap, "slides");
        slidesMotor.setRunMode(Motor.RunMode.PositionControl);
        slidesMotor.setPositionCoefficient(SLIDES_kP);
        slidesMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void periodic() {
        switch (clawState) {
            case OPENED:
                clawServo.setPosition(CLAW_OPEN);
                break;
            case CLOSED:
                clawServo.setPosition(CLAW_CLOSED);
                break;
        }

        switch (armState) {
            case IN:
                armLeft.setToPosition(ARM_IN);
                armRight.setToPosition(ARM_IN);
                break;
            case OUT:
                armLeft.setToPosition(ARM_OUT);
                armRight.setToPosition(ARM_OUT);
                break;
        }

        switch (pivotState) {
            case IN:
                armPivot.setPosition(PIVOT_IN);
                break;
            case OUT:
                armPivot.setPosition(PIVOT_OUT);
                break;
        }

        switch (slidesState) {
            case LOWERED:
                slidesMotor.setTargetPosition(SLIDES_LOWERED);
                break;
            case SPECIMEN:
                slidesMotor.setTargetPosition(SLIDES_SPECIMEN);
                break;
            case LOW_BASKET:
                slidesMotor.setTargetPosition(SLIDES_LOW_BASKET);
                break;
            case HIGH_BASKET:
                slidesMotor.setTargetPosition(SLIDES_HIGH_BASKET);
                break;
        }

        if (!slidesMotor.atTargetPosition()) {
            slidesMotor.set(1);
        } else {
            slidesMotor.set(0);
        }
    }

    public void setClawState(ClawState state) {
        clawState = state;
    }

    public void toggleClaw() {
        switch (clawState) {
            case OPENED:
                setClawState(ClawState.CLOSED);
                break;
            case CLOSED:
                setClawState(ClawState.OPENED);
                break;
        }
    }

    public ClawState getClawState() {
        return clawState;
    }

    public void setArmState(ArmState state) {
        armState = state;
    }

    public void toggleArm() {
        switch (armState) {
            case IN:
                setArmState(ArmState.OUT);
                break;
            case OUT:
                setArmState(ArmState.IN);
                break;
        }
    }

    public void setPivotState(PivotState state) {
        pivotState = state;
    }

    public void togglePivot() {
        switch (pivotState) {
            case IN:
                setPivotState(PivotState.OUT);
                break;
            case OUT:
                setPivotState(PivotState.IN);
                break;
        }
    }

    public PivotState getPivotState() {
        return pivotState;
    }

    public ArmState getArmState() {
        return armState;
    }

    public void setSlidesState(SlidesState state) {
        slidesState = state;
    }

    public void nextSlidesState() {
        setSlidesState(slidesState.next());
    }

    public void previousSlidesState() {
        setSlidesState(slidesState.previous());
    }

    public SlidesState getSlidesState() {
        return slidesState;
    }
}
