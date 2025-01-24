package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Slides Tuner", group = "Tuners")
public class SlidesTuner extends LinearOpMode {
    public static int POSITION = 0;
    public static double SLIDES_kP = 0.011, SLIDES_kD = 0.0001, SLIDES_kF = 0.1;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotorEx slidesMotor1 = (DcMotorEx) hardwareMap.dcMotor.get("slides");
        slidesMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotorEx slidesMotor2 = (DcMotorEx) hardwareMap.dcMotor.get("slides2");
        slidesMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        slidesMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        double lastError = 0;

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            double currentPosition = Math.max(-slidesMotor1.getCurrentPosition(), 0) / 8192.0 * 360;

            double error = POSITION - currentPosition;
            double derivative = (error - lastError) / timer.seconds();
            double power = (SLIDES_kP * error) + (SLIDES_kD * derivative) + SLIDES_kF;

            slidesMotor1.setPower(power);
            slidesMotor2.setPower(power);

            lastError = error;
            timer.reset();

            telemetry.addData("Encoder Position", currentPosition);
            telemetry.addData("Target Position", POSITION);
            telemetry.addLine();

            telemetry.update();
        }
    }
}
