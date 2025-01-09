package robotcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import robotcode.util.LABColor;
import robotcode.util.SampleSensor;

@Disabled
@TeleOp(name = "LAB Color Sensor Test", group = "Tests")
public class LABColorTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleSensor sensor = new SampleSensor(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            LABColor color = sensor.getColor();

            telemetry.addData("LAB", "%.3f, %.3f, %.3f", color.L, color.A, color.B);
            telemetry.addData("Distance", "%.3f", sensor.getDistance(DistanceUnit.CM));
            telemetry.addLine();

            telemetry.addData("Red Delta", sensor.deltaRed);
            telemetry.addData("Blue Delta", sensor.deltaBlue);
            telemetry.addData("Yellow Delta", sensor.deltaYellow);
            telemetry.addLine();

            telemetry.addData("Sample Detected", sensor.getSampleType());
            telemetry.addLine();

            telemetry.update();
        }
    }
}
