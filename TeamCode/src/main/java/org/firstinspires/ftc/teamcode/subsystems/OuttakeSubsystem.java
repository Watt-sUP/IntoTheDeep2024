package org.firstinspires.ftc.teamcode.subsystems;

import android.annotation.SuppressLint;
import android.util.Pair;

import androidx.annotation.NonNull;
import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.pedropathing.pathgen.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.InterpolatedAngleServo;

@Config
public class OuttakeSubsystem extends SubsystemBase {
    public static double ARM_IN = 25, ARM_OUT = 220, ARM_TRANSFER = 55, ARM_SPECIMEN = 12;
    public static double PIVOT_IN = 0.88, PIVOT_OUT = 0.4, PIVOT_SPECIMEN_DEPOSIT = 0.19, PIVOT_SPECIMEN_COLLECT = 0.46;
    public static double SLIDES_kP = 0.01, SLIDES_kI = 0.000049988, SLIDES_kD = 0.000024994, SLIDES_kF = 0, SLIDES_A = 0.8;

    private final InterpolatedAngleServo armLeft;
    private final InterpolatedAngleServo armRight;

    private final SimpleServo armPivot;
    private final SimpleServo clawServo;

    private final DcMotorEx slidesMotor1, slidesMotor2;
    private final ElapsedTime timer = new ElapsedTime();
    private ClawState clawState = null;
    private ArmState armState = null;
    private PivotState pivotState = null;
    private SlidesState slidesState = null;
    private double slidesPosition = 0;
    private double lastPosition = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private double previousFilterEstimate = 0;

    @SuppressLint("NewApi")
    public OuttakeSubsystem(HardwareMap hardwareMap) {
        clawServo = new SimpleServo(hardwareMap, "claw_servo", 0, 360);
        clawServo.setInverted(false);

        armLeft = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "arm_left", 0, 360));
        armRight = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "arm_right", 0, 360));

        armLeft.setInverted(true);
        armRight.setInverted(false);

        armLeft.generatePositions(
                new Pair<>(0.0, 0.0),
                new Pair<>(90.0, 95.0),
                new Pair<>(180.0, 190.0),
                new Pair<>(270.0, 295.0),
                new Pair<>(315.0, 348.0)
        );

        armRight.generatePositions(
                new Pair<>(0.0, 0.0),
                new Pair<>(90.0, 100.0),
                new Pair<>(180.0, 195.0),
                new Pair<>(270.0, 295.0),
                new Pair<>(315.0, 350.0)
        );

        armPivot = new SimpleServo(hardwareMap, "arm_pivot", 0, 180);

        slidesMotor1 = (DcMotorEx) hardwareMap.dcMotor.get("slides");
        slidesMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        slidesMotor2 = (DcMotorEx) hardwareMap.dcMotor.get("slides2");
        slidesMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void resetSlidesEncoder() {
        slidesMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        double maxIntegralSum = (float) (0.25 / SLIDES_kI);

        double currentPosition = Math.max(-slidesMotor1.getCurrentPosition(), 0) / 8192.0 * 360;

        double error = slidesPosition - currentPosition;
        double errorChange = error - lastError;

        double currentFilterEstimate = (SLIDES_A * previousFilterEstimate) + (1 - SLIDES_A) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        double derivative = currentFilterEstimate / timer.seconds();

        integralSum = MathFunctions.clamp(integralSum + (error * timer.seconds()), -maxIntegralSum, maxIntegralSum);

        if (slidesPosition != lastPosition) {
            integralSum = 0;
        }

        double power = (SLIDES_kP * error) + (SLIDES_kI * integralSum) + (SLIDES_kD * derivative) + (SLIDES_kF * slidesPosition);

        slidesMotor1.setPower(power);
        slidesMotor2.setPower(power);

        lastError = error;
        lastPosition = currentPosition;

        timer.reset();
    }

    public void setClawState(ClawState state) {
        clawState = state;
        clawServo.setPosition(state.position);
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
                _setArmPosition(ARM_IN);
                break;
            case OUT:
                _setArmPosition(ARM_OUT);
                break;
            case TRANSFER:
                _setArmPosition(ARM_TRANSFER);
                break;
            case SPECIMEN:
                _setArmPosition(ARM_SPECIMEN);
                break;
        }
    }

    public void _setArmPosition(double position) {
        armLeft.setToPosition(MathUtils.clamp(position, 0, 220));
        armRight.setToPosition(MathUtils.clamp(position, 0, 220));
    }

    public void toggleArm() {
        switch (armState) {
            case IN:
            case TRANSFER:
            case SPECIMEN:
                setArmState(ArmState.OUT);
                break;
            case OUT:
                setArmState(ArmState.TRANSFER);
                break;
        }
    }

    public ArmState getArmState() {
        return armState;
    }

    public double getArmPosition() {
        return armLeft.getCurrentPosition();
    }

    public void setPivotState(PivotState state) {
        pivotState = state;
        switch (pivotState) {
            case IN:
                _setPivotPosition(PIVOT_IN);
                break;
            case OUT:
                _setPivotPosition(PIVOT_OUT);
                break;
            case SPECIMEN_COLLECT:
                _setPivotPosition(PIVOT_SPECIMEN_COLLECT);
                break;
            case SPECIMEN_DEPOSIT:
                _setPivotPosition(PIVOT_SPECIMEN_DEPOSIT);
                break;
        }
    }

    public void _setPivotPosition(double position) {
        armPivot.setPosition(MathUtils.clamp(position, 0, 1));
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

    public double getPivotPosition() {
        return armPivot.getPosition();
    }

    public void setSlidesState(SlidesState state) {
        slidesState = state;
        _setSlidesPosition(state.position);
    }

    public void _setSlidesPosition(double position) {
        slidesPosition = Math.max(position, 0);
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
        slidesPosition += ticks;
    }

    public SlidesState getSlidesState() {
        return slidesState;
    }

    public Number[] getSlidesCurrent() {
        return new Number[]{
                slidesMotor1.getCurrent(CurrentUnit.AMPS),
                slidesMotor2.getCurrent(CurrentUnit.AMPS)
        };
    }

    public double getSlidesPosition() {
        return Math.max(-slidesMotor1.getCurrentPosition(), 0) / 8192.0 * 360;
    }

    public double getSlidesTarget() {
        return slidesPosition;
    }

    public enum ClawState {
        OPENED(0.55), CLOSED(0.1);

        public final double position;
        ClawState(double position) {
            this.position = position;
        }

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
        LOWERED(0),
        SPECIMEN(1625),
        LOW_BASKET(1825),
        HIGH_BASKET(3650);

        public final double position;
        SlidesState(double position) {
            this.position = position;
        }

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
}
