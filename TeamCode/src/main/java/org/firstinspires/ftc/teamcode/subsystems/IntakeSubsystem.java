package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.InterpolatedAngleServo;
import org.firstinspires.ftc.teamcode.util.InterpolatedPositionServo;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double EXTENDO_IN = 0, EXTENDO_QUARTER = 0.25, EXTENDO_HALF = 0.5, EXTENDO_THREE_QUARTERS = 0.75, EXTENDO_OUT = 1;
    public static double PIVOT_DOWN = 252, PIVOT_COLLECT = 235, PIVOT_UP = 100, PIVOT_EXTENDING = 180;
    public static double CLAW_OPEN = 0.425, CLAW_CLOSED = 0;
    public static double
            ROT_LEFT = 155,
            ROT_STRAIGHT = 110,
            ROT_RIGHT = 52.5,
            ROT_HORIZONTAL = 2.5;

    private final InterpolatedPositionServo extLeft, extRight;
    private final InterpolatedAngleServo pivLeft, pivRight;

    private final SimpleServo rotateServo, clawServo;

    private ExtendoState extendoState = null;
    private ClawState clawState = null;
    private PivotState pivotState = null;
    private RotationState rotationState = null;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        extLeft = new InterpolatedPositionServo(new SimpleServo(hardwareMap, "ext_left", 0, 220));
        extRight = new InterpolatedPositionServo(new SimpleServo(hardwareMap, "ext_right", 0, 220));

        extLeft.setInverted(true);
        extRight.setInverted(false);

        extLeft.generatePositions(
                new Pair<>(0.0, 0.0),
                new Pair<>(1.0, 0.225)
        );

        extRight.generatePositions(
                new Pair<>(0.0, 0.09),
                new Pair<>(1.0, 0.31)
        );

        pivLeft = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "int_left", 0, 360));
        pivRight = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "int_right", 0, 360));

        pivLeft.setInverted(false);
        pivRight.setInverted(true);

        pivLeft.generatePositions(
                new Pair<>(0.0, 8.0),
                new Pair<>(90.0, 112.0),
                new Pair<>(180.0, 208.0),
                new Pair<>(270.0, 310.0),
                new Pair<>(315.0, 360.0)
        );

        pivRight.generatePositions(
                new Pair<>(0.0, 8.0),
                new Pair<>(90.0, 107.0),
                new Pair<>(180.0, 208.0),
                new Pair<>(270.0, 305.0),
                new Pair<>(319.0, 360.0)
        );

        rotateServo = new SimpleServo(hardwareMap, "int_rotate", 0, 220);

        clawServo = new SimpleServo(hardwareMap, "int_claw", 0, 220);
    }

    public void nextExtendoState() {
        setExtendoState(extendoState.next());
    }

    public void previousExtendoState() {
        setExtendoState(extendoState.previous());
    }

    public ExtendoState getExtendoState() {
        return extendoState;
    }

    public void setExtendoState(ExtendoState state) {
        extendoState = state;

        switch (extendoState) {
            case IN:
                extLeft.setToPosition(EXTENDO_IN);
                extRight.setToPosition(EXTENDO_IN);
                break;
            case QUARTER:
                extLeft.setToPosition(EXTENDO_QUARTER);
                extRight.setToPosition(EXTENDO_QUARTER);
                break;
            case HALF:
                extLeft.setToPosition(EXTENDO_HALF);
                extRight.setToPosition(EXTENDO_HALF);
                break;
            case THREE_QUARTERS:
                extLeft.setToPosition(EXTENDO_THREE_QUARTERS);
                extRight.setToPosition(EXTENDO_THREE_QUARTERS);
                break;
            case OUT:
                extLeft.setToPosition(EXTENDO_OUT);
                extRight.setToPosition(EXTENDO_OUT);
                break;
        }
    }

    public void togglePivot() {
        switch (pivotState) {
            case DOWN:
            case EXTENDING:
                setPivotState(PivotState.COLLECT);
                break;
            case UP:
            case COLLECT:
                setPivotState(PivotState.DOWN);
                break;
        }
    }

    public PivotState getPivotState() {
        return pivotState;
    }

    public void setPivotState(PivotState state) {
        pivotState = state;

        switch (pivotState) {
            case DOWN:
                pivLeft.setToPosition(PIVOT_DOWN);
                pivRight.setToPosition(PIVOT_DOWN);
                break;
            case COLLECT:
                pivLeft.setToPosition(PIVOT_COLLECT);
                pivRight.setToPosition(PIVOT_COLLECT);
                break;
            case EXTENDING:
                pivLeft.setToPosition(PIVOT_EXTENDING);
                pivRight.setToPosition(PIVOT_EXTENDING);
                setRotation(RotationState.STRAIGHT);
                break;
            case UP:
                pivLeft.setToPosition(PIVOT_UP);
                pivRight.setToPosition(PIVOT_UP);
                setRotation(RotationState.STRAIGHT);
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

    public void nextRotation() {
        setRotation(rotationState.next());
    }

    public void previousRotation() {
        setRotation(rotationState.previous());
    }

    public RotationState getRotation() {
        return rotationState;
    }

    public void setRotation(RotationState state) {
        if (pivotState == PivotState.EXTENDING || pivotState == PivotState.UP)
            return;

        rotationState = state;

        switch (rotationState) {
            case LEFT:
                rotateServo.turnToAngle(ROT_LEFT);
                break;
            case STRAIGHT:
                rotateServo.turnToAngle(ROT_STRAIGHT);
                break;
            case RIGHT:
                rotateServo.turnToAngle(ROT_RIGHT);
                break;
            case HORIZONTAL:
                rotateServo.turnToAngle(ROT_HORIZONTAL);
                break;
        }
    }

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

    public enum ExtendoState {
        IN,
        QUARTER,
        HALF,
        THREE_QUARTERS,
        OUT;

        @NonNull
        public String toString() {
            switch (this) {
                case IN:
                    return "In";
                case QUARTER:
                    return "Quarter";
                case HALF:
                    return "Half";
                case THREE_QUARTERS:
                    return "Three Quarters";
                case OUT:
                    return "Out";
            }
            return "";
        }

        public ExtendoState next() {
            switch (this) {
                case IN:
                    return QUARTER;
                case QUARTER:
                    return HALF;
                case HALF:
                    return THREE_QUARTERS;
                case THREE_QUARTERS:
                case OUT:
                    return OUT;
            }
            return IN;
        }

        public ExtendoState previous() {
            switch (this) {
                case OUT:
                    return THREE_QUARTERS;
                case THREE_QUARTERS:
                    return HALF;
                case HALF:
                    return QUARTER;
                case QUARTER:
                case IN:
                    return IN;
            }
            return IN;
        }
    }

    public enum PivotState {
        DOWN,
        COLLECT,
        EXTENDING,
        UP;

        @NonNull
        public String toString() {
            switch (this) {
                case DOWN:
                    return "Down";
                case UP:
                    return "Up";
                case COLLECT:
                    return "Collect";
                case EXTENDING:
                    return "Extending";
            }
            return "";
        }
    }

    public enum RotationState {
        STRAIGHT,
        HORIZONTAL,
        LEFT,
        RIGHT;

        @NonNull
        public String toString() {
            switch (this) {
                case STRAIGHT:
                    return "Straight";
                case HORIZONTAL:
                    return "Horizontal";
                case LEFT:
                    return "Left";
                case RIGHT:
                    return "Right";
            }
            return "";
        }

        public RotationState next() {
            switch (this) {
                case HORIZONTAL:
                    return LEFT;
                case LEFT:
                    return STRAIGHT;
                case STRAIGHT:
                    return RIGHT;
                case RIGHT:
                    return HORIZONTAL;
            }
            return STRAIGHT;
        }

        public RotationState previous() {
            switch (this) {
                case HORIZONTAL:
                    return RIGHT;
                case RIGHT:
                    return STRAIGHT;
                case STRAIGHT:
                    return LEFT;
                case LEFT:
                    return HORIZONTAL;
            }
            return STRAIGHT;
        }
    }
}
