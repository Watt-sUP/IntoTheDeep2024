package org.firstinspires.ftc.teamcode.tests;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.InterpolatedAngleServo;

@Config
@TeleOp(name = "Intake Pivot Test", group = "Tests")
public class IntakePivotTest extends LinearOpMode {
    public static double DEGREES = 0;

    @Override
    public void runOpMode() {
        InterpolatedAngleServo pivLeft = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "int_left", 0, 270));
        InterpolatedAngleServo pivRight = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "int_right", 0, 270));

        pivLeft.setInverted(true);
        pivRight.setInverted(false);

        pivLeft.generatePositions(
                new Pair<>(0.0, 7.0),
                new Pair<>(90.0, 109.0),
                new Pair<>(180.0, 214.0),
                new Pair<>(215.0, 270.0)
        );

        pivRight.generatePositions(
                new Pair<>(0.0, 24.0),
                new Pair<>(90.0, 123.0),
                new Pair<>(180.0, 222.0),
                new Pair<>(210.0, 270.0)
        );

        waitForStart();

        while (opModeIsActive()) {
            pivLeft.setToPosition(DEGREES);
            pivRight.setToPosition(DEGREES);

            telemetry.addData("Pivot Left Position", pivLeft.getCurrentPosition());
            telemetry.addData("Pivot Right Position", pivRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
