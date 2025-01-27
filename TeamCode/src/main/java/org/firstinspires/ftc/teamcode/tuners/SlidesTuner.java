package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "Slides Tuner", group = "Tuners")
public class SlidesTuner extends LinearOpMode {
    public static int POSITION = 0;
    public static PIDFCoefficients SLIDES_PIDF = new PIDFCoefficients(0.01, 0, 0.0001, 0.1);
    public static double CURRENT_THRESHOLD = 6;
    public static boolean ENABLE_STALL_PAUSE = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(16);

        // NOTE: Apparently getCurrent and voltage sensor readings aren't in bulk, isOverCurrent is though.
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();
        Timing.Timer voltageCooldown = new Timing.Timer(500, TimeUnit.MILLISECONDS);

        DcMotorEx slidesMotor1 = hardwareMap.get(DcMotorEx.class, "slides");
        slidesMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotorEx slidesMotor2 = hardwareMap.get(DcMotorEx.class, "slides2");
        slidesMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        slidesMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (ENABLE_STALL_PAUSE) {
            slidesMotor1.setCurrentAlert(CURRENT_THRESHOLD, CurrentUnit.AMPS);
            slidesMotor2.setCurrentAlert(CURRENT_THRESHOLD, CurrentUnit.AMPS);
        }

        waitForStart();

        double lastError = 0;
        double voltage = voltageSensor.getVoltage();
        ElapsedTime timer = new ElapsedTime();
        voltageCooldown.start();

        while (opModeIsActive()) {
            double currentPosition = Math.max(-slidesMotor1.getCurrentPosition(), 0) / 8192.0 * 360;
            double error = POSITION - currentPosition;
            double derivative = (error - lastError) / timer.seconds();

            if (voltageCooldown.done()) { // Update voltage every 500ms (margin of error can be ~1V)
                voltage = voltageSensor.getVoltage();
                voltageCooldown.start();
            }

            // Account for asymmetric response
            double PID_output = Range.clip(SLIDES_PIDF.p * error + SLIDES_PIDF.d * derivative, SLIDES_PIDF.f - 1, 1 - SLIDES_PIDF.f);
            double power = (PID_output + SLIDES_PIDF.f) * Math.min(12.0 / voltage, 1); // Account for voltage

            if (ENABLE_STALL_PAUSE && slidesMotor1.isOverCurrent()) {
                slidesMotor1.setPower(SLIDES_PIDF.f);
                slidesMotor2.setPower(SLIDES_PIDF.f);
            } else {
                slidesMotor1.setPower(power);
                slidesMotor2.setPower(power);
            }

            lastError = error;
            timer.reset();

            telemetry.addData("Encoder Position", currentPosition);
            telemetry.addData("Target Position", POSITION);
            telemetry.addLine();
            telemetry.addData("Motor Current", slidesMotor1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Motor Stalled", slidesMotor1.isOverCurrent() ? "Yes" : "No");
            telemetry.addData("Robot Voltage", voltage);

            telemetry.update();
        }
    }
}
