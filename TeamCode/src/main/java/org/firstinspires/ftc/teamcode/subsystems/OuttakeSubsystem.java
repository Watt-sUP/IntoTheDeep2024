package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        TRANSFER,
        OUT;

        @NonNull
        public String toString() {
            switch (this) {
                case IN:
                    return "In";
                case SPECIMEN:
                    return "Specimen";
                case TRANSFER:
                    return "Transfer";
                case OUT:
                    return "Out";
            }
            return "";
        }
    }

    public enum PivotState {
        IN,
        SPECIMEN_DEPOSIT,
        SPECIMEN_COLLECT,
        OUT;

        @NonNull
        public String toString() {
            switch (this) {
                case IN:
                    return "In";
                case SPECIMEN_DEPOSIT:
                    return "Specimen Deposit";
                case SPECIMEN_COLLECT:
                    return "Specimen Collect";
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
                case HIGH_BASKET:
                    return HIGH_BASKET;
            }
            return LOWERED;
        }

        public SlidesState previous() {
            switch (this) {
                case HIGH_BASKET:
                    return LOW_BASKET;
                case LOW_BASKET:
                case LOWERED:
                    return LOWERED;
            }
            return LOWERED;
        }
    }

    public static double CLAW_CLOSED = 0, CLAW_OPEN = 0.55;
    public static double ARM_IN = 30, ARM_OUT = 220, ARM_TRANSFER = 120, ARM_SPECIMEN = 15;
    public static double PIVOT_IN = 0.87, PIVOT_OUT = 0.4, PIVOT_SPECIMEN_DEPOSIT = 0.84, PIVOT_SPECIMEN_COLLECT = 0.45;

    public static int SLIDES_LOWERED = 0,
            SLIDES_SPECIMEN = 145,
            SLIDES_LOW_BASKET = 500,
            SLIDES_HIGH_BASKET = 1300;

    private final InterpolatedAngleServo armLeft;
    private final InterpolatedAngleServo armRight;
    private final SimpleServo armPivot;

    private final SimpleServo clawServo;

    private final DcMotor slidesMotor1, slidesMotor2;

    private ClawState clawState = null;
    private ArmState armState = null;
    private PivotState pivotState = null;
    private SlidesState slidesState = null;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        clawServo = new SimpleServo(hardwareMap, "claw_servo", 0, 360);
        clawServo.setInverted(false);

        armLeft = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "arm_left", 0, 220));
        armRight = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "arm_right", 0, 220));

        armLeft.setInverted(true);
        armRight.setInverted(false);

        armLeft.generatePositions(
                new Pair<>(0.0, 0.0),
                new Pair<>(90.0, 100.0),
                new Pair<>(180.0, 196.0),
                new Pair<>(220.0, 220.0)
        );

        armRight.generatePositions(
                new Pair<>(0.0, 0.0),
                new Pair<>(90.0, 108.0),
                new Pair<>(180.0, 198.0),
                new Pair<>(220.0, 215.0)
        );

        armPivot = new SimpleServo(hardwareMap, "arm_pivot", 0, 180);

        slidesMotor1 = hardwareMap.dcMotor.get("slides");
        slidesMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        slidesMotor2 = hardwareMap.dcMotor.get("slides2");
        slidesMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slidesMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor slidesMotor1 = hardwareMap.dcMotor.get("slides");
        slidesMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor slidesMotor2 = hardwareMap.dcMotor.get("slides2");
        slidesMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        slidesMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesMotor1.setTargetPosition(SLIDES_LOWERED);
        slidesMotor2.setTargetPosition(SLIDES_LOWERED);

        slidesMotor1.setPower(1);
        slidesMotor2.setPower(1);

        slidesMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    public double getClawPosition() {
        return clawServo.getPosition();
    }

    public void setArmState(ArmState state) {
        armState = state;
        switch (armState) {
            case IN:
                armLeft.setToPosition(ARM_IN);
                armRight.setToPosition(ARM_IN);
                break;
            case OUT:
                armLeft.setToPosition(ARM_OUT);
                armRight.setToPosition(ARM_OUT);
                break;
            case TRANSFER:
                armLeft.setToPosition(ARM_TRANSFER);
                armRight.setToPosition(ARM_TRANSFER);
                break;
            case SPECIMEN:
                armLeft.setToPosition(ARM_SPECIMEN);
                armRight.setToPosition(ARM_SPECIMEN);
                break;
        }
    }

    public void toggleArm() {
        switch (armState) {
            case IN:
            case SPECIMEN:
                setArmState(ArmState.OUT);
                break;
            case OUT:
                setArmState(ArmState.IN);
                break;
        }
    }

    public ArmState getArmState() {
        return armState;
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
            case SPECIMEN_COLLECT:
                armPivot.setPosition(PIVOT_SPECIMEN_COLLECT);
                break;
            case SPECIMEN_DEPOSIT:
                armPivot.setPosition(PIVOT_SPECIMEN_DEPOSIT);
                break;
        }
    }

    public void togglePivot() {
        switch (pivotState) {
            case IN:
            case SPECIMEN_COLLECT:
                setPivotState(PivotState.OUT);
                break;
            case OUT:
            case SPECIMEN_DEPOSIT:
                setPivotState(PivotState.IN);
                break;
        }
    }

    public PivotState getPivotState() {
        return pivotState;
    }


    public void setSlidesState(SlidesState state) {
        slidesState = state;
        switch (slidesState) {
            case LOWERED:
                slidesMotor1.setTargetPosition(SLIDES_LOWERED);
                slidesMotor2.setTargetPosition(SLIDES_LOWERED);
                break;
            case LOW_BASKET:
                slidesMotor1.setTargetPosition(SLIDES_LOW_BASKET);
                slidesMotor2.setTargetPosition(SLIDES_LOW_BASKET);
                break;
            case HIGH_BASKET:
                slidesMotor1.setTargetPosition(SLIDES_HIGH_BASKET);
                slidesMotor2.setTargetPosition(SLIDES_HIGH_BASKET);
                break;
            case SPECIMEN:
                slidesMotor1.setTargetPosition(SLIDES_SPECIMEN);
                slidesMotor2.setTargetPosition(SLIDES_SPECIMEN);
                break;
        }
    }

    public void nextSlidesState() {
        if (slidesState == SlidesState.SPECIMEN) {
            setSlidesState(SlidesState.LOW_BASKET);
            return;
        }
        setSlidesState(slidesState.next());
    }

    public void previousSlidesState() {
        if (slidesState == SlidesState.SPECIMEN) {
            setSlidesState(SlidesState.LOWERED);
            return;
        }
        setSlidesState(slidesState.previous());
    }

    public void adjustSlides(int ticks) {
        slidesMotor1.setTargetPosition(slidesMotor1.getTargetPosition() + ticks);
        slidesMotor2.setTargetPosition(slidesMotor2.getTargetPosition() + ticks);
    }

    public void resetSlidesEncoder() {
        slidesMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setSlidesState(slidesState);

        slidesMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public SlidesState getSlidesState() {
        return slidesState;
    }
}
