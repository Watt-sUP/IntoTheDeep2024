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
        DOWN,
        UP;

        @NonNull
        public String toString() {
            switch (this) {
                case DOWN:
                    return "Down";
                case UP:
                    return "Up";
            }
            return "";
        }
    }

    public static double EXTENDO_IN = 0, EXTENDO_OUT = 1;
    public static double PIVOT_DOWN = 90, PIVOT_UP = 270;
    public static double CLAW_OPEN = 0, CLAW_CLOSED = 0.4;

    private final InterpolatedPositionServo extLeft, extRight;
    private final InterpolatedAngleServo pivLeft, pivRight;
    private final SimpleServo rotateServo, clawServo;

    private ExtendoState extendoState = ExtendoState.IN;
    private ClawState clawState = ClawState.CLOSED;
    private PivotState pivotState = PivotState.UP;


    public IntakeSubsystem(HardwareMap hardwareMap) {
        extLeft = new InterpolatedPositionServo(new SimpleServo(hardwareMap, "ext_left", 0, 220));
        extRight = new InterpolatedPositionServo(new SimpleServo(hardwareMap, "ext_right", 0, 220));

        extLeft.setInverted(false);
        extRight.setInverted(true);

        extLeft.generatePositions(
                new Pair<>(0.0, 0.53),
                new Pair<>(0.5, 0.735),
                new Pair<>(1.0, 0.92)
        );

        extRight.generatePositions(
                new Pair<>(0.0, 0.49),
                new Pair<>(0.5, 0.66),
                new Pair<>(1.0, 0.83)
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
            case OUT:
                extLeft.setToPosition(EXTENDO_OUT);
                extRight.setToPosition(EXTENDO_OUT);
                break;
        }
    }

    public void toggleExtendo() {
        switch (extendoState) {
            case IN:
                setExtendoState(ExtendoState.OUT);
                break;
            case OUT:
                setExtendoState(ExtendoState.IN);
                break;
        }
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
            case UP:
                pivLeft.setToPosition(PIVOT_UP);
                pivRight.setToPosition(PIVOT_UP);
                break;
        }
    }

    public void togglePivot() {
        switch (pivotState) {
            case DOWN:
                setPivotState(PivotState.UP);
                break;
            case UP:
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

    public void setRotation(double angle) {
        rotateServo.turnToAngle(angle);
    }
}
