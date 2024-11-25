package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        SPECIMEN,
        OUT;

        @NonNull
        public String toString() {
            switch (this) {
                case IN:
                    return "In";
                case SPECIMEN:
                    return "Specimen";
                case OUT:
                    return "Out";
            }
            return "";
        }
    }

    public enum PivotState {
        IN,
        SPECIMEN,
        OUT;

        @NonNull
        public String toString() {
            switch (this) {
                case IN:
                    return "In";
                case SPECIMEN:
                    return "Specimen";
                case OUT:
                    return "Out";
            }
            return "";
        }
    }

    public enum SlidesState {
        LOWERED,
        LOW_BASKET,
        SPECIMEN,
        HIGH_BASKET;

        @NonNull
        public String toString() {
            switch (this) {
                case LOWERED:
                    return "Lowered";
                case LOW_BASKET:
                    return "Low Basket";
                case SPECIMEN:
                    return "Specimen";
                case HIGH_BASKET:
                    return "High Basket";
            }
            return "";
        }

        public SlidesState next() {
            switch (this) {
                case LOWERED:
                    return LOW_BASKET;
                case LOW_BASKET:
                    return SPECIMEN;
                case HIGH_BASKET:
                case SPECIMEN:
                    return HIGH_BASKET;
            }
            return LOWERED;
        }

        public SlidesState previous() {
            switch (this) {
                case LOWERED:
                case LOW_BASKET:
                    return LOWERED;
                case SPECIMEN:
                    return LOW_BASKET;
                case HIGH_BASKET:
                    return SPECIMEN;
            }
            return LOWERED;
        }
    }

    public static double CLAW_CLOSED = 0, CLAW_OPEN = 0.55;
    public static double ARM_IN = 10, ARM_SPECIMEN = 100, ARM_OUT = 220;
    public static double PIVOT_IN = 0.87, PIVOT_OUT = 0.4, PIVOT_SPECIMEN = 0.6;

    public static int SLIDES_LOWERED = 0,
            SLIDES_LOW_BASKET = 650,
            SLIDES_SPECIMEN = 800,
            SLIDES_HIGH_BASKET = 1500;

    private final InterpolatedAngleServo armLeft;
    private final InterpolatedAngleServo armRight;
    private final SimpleServo armPivot;

    private final SimpleServo clawServo;
    private final DcMotor slidesMotor;

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

        slidesMotor = hardwareMap.dcMotor.get("slides");
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesMotor.setPower(1);
        slidesMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setClawState(ClawState state) {

        clawState = state;
        switch (clawState) {
            case OPENED:
                clawServo.setPosition(CLAW_OPEN);
                break;
            case CLOSED:
                clawServo.setPosition(CLAW_CLOSED);
                break;
        }
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
        switch (armState) {
            case IN:
                armLeft.setToPosition(ARM_IN);
                armRight.setToPosition(ARM_IN);
                break;
            case SPECIMEN:
                armLeft.setToPosition(ARM_SPECIMEN);
                armRight.setToPosition(ARM_SPECIMEN);
                break;
            case OUT:
                armLeft.setToPosition(ARM_OUT);
                armRight.setToPosition(ARM_OUT);
                break;
        }
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
        switch (pivotState) {
            case IN:
                armPivot.setPosition(PIVOT_IN);
                break;
            case OUT:
                armPivot.setPosition(PIVOT_OUT);
                break;
            case SPECIMEN:
                armPivot.setPosition(PIVOT_SPECIMEN);
                break;
        }
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
        switch (slidesState) {
            case LOWERED:
                slidesMotor.setTargetPosition(SLIDES_LOWERED);
                break;
            case LOW_BASKET:
                slidesMotor.setTargetPosition(SLIDES_LOW_BASKET);
                break;
            case HIGH_BASKET:
                slidesMotor.setTargetPosition(SLIDES_HIGH_BASKET);
                break;
            case SPECIMEN:
                slidesMotor.setTargetPosition(SLIDES_SPECIMEN);
                break;
        }
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
