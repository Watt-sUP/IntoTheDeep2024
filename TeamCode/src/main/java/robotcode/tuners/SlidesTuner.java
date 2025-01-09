package robotcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
@TeleOp(name = "Slides Tuner", group = "Tuners")
public class SlidesTuner extends LinearOpMode {
    public static int POSITION = 0;
    public static double SLIDES_kP = 0.01, SLIDES_kI = 0.000049988, SLIDES_kD = 0.000024994, SLIDES_kF = 0, SLIDES_A = 0.8;

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

        slidesMotor1.setCurrentAlert(6.2, CurrentUnit.AMPS);
        slidesMotor2.setCurrentAlert(6.2, CurrentUnit.AMPS);

        waitForStart();

        int lastPosition = POSITION;
        double integralSum = 0;

        double lastError = 0;

        double maxIntegralSum;

        double previousFilterEstimate = 0;
        double currentFilterEstimate;

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()) {
            maxIntegralSum = (float) (0.25 / SLIDES_kI);

            int currentPosition = Math.max(-slidesMotor1.getCurrentPosition(), 0) / 100;

            double error = POSITION - currentPosition;
            double errorChange = error - lastError;

            currentFilterEstimate = (SLIDES_A * previousFilterEstimate) + (1 - SLIDES_A) * errorChange;
            previousFilterEstimate = currentFilterEstimate;

            double derivative = currentFilterEstimate / timer.seconds();

            integralSum = integralSum + (error * timer.seconds());

            if (integralSum > maxIntegralSum) {
                integralSum = maxIntegralSum;
            }

            if (integralSum < -maxIntegralSum) {
                integralSum = -maxIntegralSum;
            }

            if (POSITION != lastPosition) {
                integralSum = 0;
            }

            double power = (SLIDES_kP * error) + (SLIDES_kI * integralSum) + (SLIDES_kD * derivative) + (SLIDES_kF * POSITION);

            slidesMotor1.setPower(power);
            slidesMotor2.setPower(power);

            lastError = error;
            lastPosition = currentPosition;

            timer.reset();

            telemetry.addData("Encoder Position", currentPosition);
            telemetry.addData("Target Position", POSITION);
            telemetry.addLine();

            if (slidesMotor1.isOverCurrent() || slidesMotor2.isOverCurrent()) {
                telemetry.addData("Current", "%.2f, %.2f !! OVERCURRENT !!", slidesMotor1.getCurrent(CurrentUnit.AMPS), slidesMotor2.getCurrent(CurrentUnit.AMPS));
            } else {
                telemetry.addData("Current", "%.2f, %.2f", slidesMotor1.getCurrent(CurrentUnit.AMPS), slidesMotor2.getCurrent(CurrentUnit.AMPS));
            }

            telemetry.update();
        }
    }
}
