package robotcode.tests;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import robotcode.util.InterpolatedAngleServo;

@Config
@TeleOp(name = "Arm Test", group = "Tests")
public class ArmTest extends LinearOpMode {
    public static double DEGREES = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        InterpolatedAngleServo armLeft = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "arm_left", 0, 220));
        InterpolatedAngleServo armRight = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "arm_right", 0, 220));

        armLeft.setInverted(true);
        armRight.setInverted(false);

        armLeft.generatePositions(
                new Pair<>(0.0, 0.0),
                new Pair<>(90.0, 100.0),
                new Pair<>(180.0, 196.0),
                new Pair<>(220.0, 220.0)
        );

        armRight.generatePositions(
                new Pair<>(0.0, 0.0),
                new Pair<>(90.0, 108.0),
                new Pair<>(180.0, 198.0),
                new Pair<>(220.0, 215.0)
        );

        waitForStart();

        while (opModeIsActive()) {
            armLeft.setToPosition(DEGREES);
            armRight.setToPosition(DEGREES);

            telemetry.addData("Arm Left Position", armLeft.getCurrentPosition());
            telemetry.addData("Arm Right Position", armRight.getCurrentPosition());
            telemetry.update();
        }
    }
}