package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.InterpolatedAngleServo;
import org.firstinspires.ftc.teamcode.util.InterpolatedPositionServo;

@Config
public class IntakeSubsystem extends SubsystemBase {
    public static double EXTENDO_IN = 0, EXTENDO_QUARTER = 0.25, EXTENDO_HALF = 0.5, EXTENDO_THREE_QUARTERS = 0.75, EXTENDO_OUT = 1;
    public static double PIVOT_DOWN = 180, PIVOT_COLLECT = 135, PIVOT_UP = 0, PIVOT_EXTENDING = 80;
    public static double CLAW_OPEN = 0.5, CLAW_CLOSED = 0.05;
    public static double
            ROT_LEFT = 0.7,
            ROT_STRAIGHT = 0.53,
            ROT_RIGHT = 0.35,
            ROT_HORIZONTAL = 0.15;

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
                new Pair<>(0.25, 0.075),
                new Pair<>(0.5, 0.15),
                new Pair<>(0.75, 0.255),
                new Pair<>(1.0, 0.3)
        );

        extRight.generatePositions(
                new Pair<>(0.0, 0.0),
                new Pair<>(0.25, 0.0875),
                new Pair<>(0.5, 0.175),
                new Pair<>(0.75, 0.2625),
                new Pair<>(1.0, 0.35)
        );

        pivLeft = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "int_left", 0, 185));
        pivRight = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "int_right", 0, 185));

        pivLeft.setInverted(false);
        pivRight.setInverted(true);

        pivLeft.generatePositions(
                new Pair<>(0.0, 0.0),
                new Pair<>(90.0, 100.0),
                new Pair<>(135.0, 130.0),
                new Pair<>(180.0, 185.0)
        );

        pivRight.generatePositions(
                new Pair<>(0.0, 0.0),
                new Pair<>(90.0, 105.0),
                new Pair<>(135.0, 130.0),
                new Pair<>(180.0, 185.0)
        );

        rotateServo = new SimpleServo(hardwareMap, "int_rotate", 0, 270);

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
                _setExtendoPosition(EXTENDO_IN);
                break;
            case QUARTER:
                _setExtendoPosition(EXTENDO_QUARTER);
                break;
            case HALF:
                _setExtendoPosition(EXTENDO_HALF);
                break;
            case THREE_QUARTERS:
                _setExtendoPosition(EXTENDO_THREE_QUARTERS);
                break;
            case OUT:
                _setExtendoPosition(EXTENDO_OUT);
                break;
        }
    }

    public void _setExtendoPosition(double pos) {
        extLeft.setToPosition(pos);
        extRight.setToPosition(pos);
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
        rotationState = state;

        switch (rotationState) {
            case LEFT:
                rotateServo.setPosition(ROT_LEFT);
                break;
            case STRAIGHT:
                rotateServo.setPosition(ROT_STRAIGHT);
                break;
            case RIGHT:
                rotateServo.setPosition(ROT_RIGHT);
                break;
            case HORIZONTAL:
                rotateServo.setPosition(ROT_HORIZONTAL);
                break;
        }
    }

    public enum ClawState {
        OPENED,
        CLOSED;
    }

    public enum ExtendoState {
        IN,
        QUARTER,
        HALF,
        THREE_QUARTERS,
        OUT;

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
    }

    public enum RotationState {
        STRAIGHT,
        HORIZONTAL,
        LEFT,
        RIGHT;

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
