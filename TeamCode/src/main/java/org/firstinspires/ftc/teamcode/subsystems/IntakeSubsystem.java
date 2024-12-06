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

    public static double EXTENDO_IN = 0, EXTENDO_QUARTER = 0.25, EXTENDO_HALF = 0.5, EXTENDO_THREE_QUARTERS = 0.75, EXTENDO_OUT = 1;
    public static double PIVOT_DOWN = 80, PIVOT_COLLECT = 120, PIVOT_UP = 250, PIVOT_EXTENDING = 165;
    public static double CLAW_OPEN = 0, CLAW_CLOSED = 0.52;
    public static double
            ROT_LEFT = 145,
            ROT_STRAIGHT = 95,
            ROT_RIGHT = 45,
            ROT_HORIZONTAL = 0;

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

        extLeft.setInverted(false);
        extRight.setInverted(true);

        extLeft.generatePositions(
                new Pair<>(0.0, 0.5),
                new Pair<>(0.5, 0.67),
                new Pair<>(1.0, 0.85)
        );

        extRight.generatePositions(
                new Pair<>(0.0, 0.43),
                new Pair<>(0.5, 0.625),
                new Pair<>(1.0, 0.82)
        );

        pivLeft = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "int_left", 0, 1800));
        pivRight = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "int_right", 0, 1800));

        pivLeft.setInverted(false);
        pivRight.setInverted(true);

        pivLeft.generatePositions(
                new Pair<>(0.0, 5.0),
                new Pair<>(90.0, 94.0),
                new Pair<>(180.0, 190.0),
                new Pair<>(270.0, 286.0),
                new Pair<>(360.0, 374.0)
        );

        pivRight.generatePositions(
                new Pair<>(0.0, 3.0),
                new Pair<>(90.0, 90.0),
                new Pair<>(180.0, 184.0),
                new Pair<>(270.0, 283.0),
                new Pair<>(360.0, 380.0)
        );

        rotateServo = new SimpleServo(hardwareMap, "int_rotate", 0, 220);

        clawServo = new SimpleServo(hardwareMap, "int_claw", 0, 220);
        clawServo.setInverted(true);
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

    public void nextExtendoState() {
        setExtendoState(extendoState.next());
    }

    public void previousExtendoState() {
        setExtendoState(extendoState.previous());
    }

    public ExtendoState getExtendoState() {
        return extendoState;
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
                break;
            case UP:
                pivLeft.setToPosition(PIVOT_UP);
                pivRight.setToPosition(PIVOT_UP);
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

    public void setRotation(RotationState state) {
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

    public void nextRotation() {
        setRotation(rotationState.next());
    }

    public void previousRotation() {
        setRotation(rotationState.previous());
    }

    public RotationState getRotation() {
        return rotationState;
    }
}
