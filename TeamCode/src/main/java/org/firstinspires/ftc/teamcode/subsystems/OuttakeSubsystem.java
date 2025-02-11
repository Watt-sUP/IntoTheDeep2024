package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Pair;

import androidx.annotation.NonNull;
import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.InterpolatedAngleServo;

import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;
import java.util.stream.Stream;

@Config
public class OuttakeSubsystem extends SubsystemBase {
    public static double ARM_TRANSFER = 55, ARM_OUT = 180, ARM_IN = 80, ARM_SPECIMEN_COLLECT = 44, ARM_SPECIMEN_DEPOSIT = 198;
    public static double PIVOT_IN = 0.99, PIVOT_OUT = 0.28, PIVOT_SPECIMEN_DEPOSIT = 0.19, PIVOT_SPECIMEN_COLLECT = 0.47;
    public static PIDFCoefficients SLIDES_PIDF = new PIDFCoefficients(0.005, 0, 0.00003, 0.05);

    private final InterpolatedAngleServo armLeft, armRight;
    private final SimpleServo armPivot, clawServo;

    private final List<DcMotorEx> motors;
    private final VoltageSensor voltageSensor;
    private final ElapsedTime timer = new ElapsedTime();
    private final Timing.Timer voltageCooldown = new Timing.Timer(500, TimeUnit.MILLISECONDS);

    private ClawState clawState;
    private ArmState armState;
    private PivotState pivotState;
    private SlidesState slidesState;
    private double slidesPosition, lastError, voltage;

    private boolean hanging = false;

    public OuttakeSubsystem(HardwareMap hardwareMap) {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        clawServo = new SimpleServo(hardwareMap, "claw_servo", 0, 360);
        clawServo.setInverted(false);

        armLeft = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "arm_left", 0, 220));
        armRight = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "arm_right", 0, 220));

        armLeft.setInverted(false);
        armRight.setInverted(true);

        armLeft.generatePositions(
                new Pair<>(0.0, 9.0),
                new Pair<>(90.0, 108.0),
                new Pair<>(180.0, 202.0),
                new Pair<>(198.0, 220.0)
        );

        armRight.generatePositions(
                new Pair<>(0.0, 2.0),
                new Pair<>(90.0, 98.0),
                new Pair<>(180.0, 193.0),
                new Pair<>(205.0, 220.0)
        );

        armPivot = new SimpleServo(hardwareMap, "arm_pivot", 0, 180);

        motors = Stream.of("Up", "Down", "Left", "Right")
                .map(id -> hardwareMap.get(DcMotorEx.class, "slides" + id))
                .collect(Collectors.toList());

        motors.get(1).setDirection(DcMotorSimple.Direction.REVERSE);
        motors.get(3).setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void resetSlidesEncoder() {
        motors.get(0).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(0).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        double error = slidesPosition - this.getSlidesPosition();
        double derivative = (error - lastError) / timer.seconds();

        if (voltageCooldown.done() || !voltageCooldown.isTimerOn()) {
            voltage = voltageSensor.getVoltage();
            voltageCooldown.start();
        }

        // Account for asymmetric response
        double PID_output = Range.clip(SLIDES_PIDF.p * error + SLIDES_PIDF.d * derivative, SLIDES_PIDF.f - 1, 1 - SLIDES_PIDF.f);
        double power = (PID_output + SLIDES_PIDF.f) * Math.min(12.0 / voltage, 1); // Account for voltage

        motors.forEach(motor -> motor.setPower(power));
        
        lastError = error;
        timer.reset();
    }

    public void setClawState(@NonNull ClawState state) {
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
            case TRANSFER:
                _setArmPosition(ARM_TRANSFER);
                break;
            case OUT:
                _setArmPosition(ARM_OUT);
                break;
            case IN:
                _setArmPosition(ARM_IN);
                break;
            case SPECIMEN_COLLECT:
                _setArmPosition(ARM_SPECIMEN_COLLECT);
                break;
            case SPECIMEN_DEPOSIT:
                _setArmPosition(ARM_SPECIMEN_DEPOSIT);
                break;
        }
    }

    public void _setArmPosition(double position) {
        armLeft.setToPosition(MathUtils.clamp(position, 0, 310));
        armRight.setToPosition(MathUtils.clamp(position, 0, 310));
    }

    public void toggleArm() {
        switch (armState) {
            case TRANSFER:
            case IN:
                setArmState(ArmState.OUT);
                break;
            case SPECIMEN_COLLECT:
                setArmState(ArmState.SPECIMEN_DEPOSIT);
                break;
            case OUT:
            case SPECIMEN_DEPOSIT:
                setArmState(ArmState.IN);
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

    public void setSlidesState(@NonNull SlidesState state) {
        slidesState = state;
        _setSlidesPosition(state.position);
    }

    public void _setSlidesPosition(double position) {
        slidesPosition = Math.max(position, 0);
    }

    public void nextSlidesState() {
        switch (slidesState) {
            case LOWERED:
            case SPECIMEN:
                setSlidesState(SlidesState.LOW_BASKET);
                break;
            default:
                setSlidesState(SlidesState.HIGH_BASKET);
        }
    }

    public void previousSlidesState() {
        if (slidesState == SlidesState.HIGH_BASKET)
            setSlidesState(SlidesState.LOW_BASKET);
        else setSlidesState(SlidesState.LOWERED);
    }

    public void adjustSlides(double val) {
        slidesPosition += val;
    }

    public SlidesState getSlidesState() {
        return slidesState;
    }

    public double getSlidesPosition() {
        return Math.max(-motors.get(0).getCurrentPosition(), 0) / 8192.0 * 360;
    }

    public double getSlidesTarget() {
        return slidesPosition;
    }

    public void setHanging(boolean val) {
        hanging = val;
    }

    public boolean isHanging() {
        return hanging;
    }

    public enum ClawState {
        OPENED(0.67), CLOSED(0.38);

        public final double position;

        ClawState(double position) {
            this.position = position;
        }
    }

    public enum ArmState {
        TRANSFER,
        SPECIMEN_COLLECT,
        SPECIMEN_DEPOSIT,
        IN,
        OUT;
    }

    public enum PivotState {
        IN,
        SPECIMEN_DEPOSIT,
        SPECIMEN_COLLECT,
        OUT;
    }

    public enum SlidesState {
        LOWERED(0),
        SPECIMEN(1300),
        LOW_BASKET(1100),
        HIGH_BASKET(2900),
        HANG_PREPARE(3300),
        HANG(2000);

        public final double position;

        SlidesState(double position) {
            this.position = position;
        }
    }
}
