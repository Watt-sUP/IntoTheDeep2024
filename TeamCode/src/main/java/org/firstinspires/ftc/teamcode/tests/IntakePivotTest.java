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
        InterpolatedAngleServo pivLeft = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "int_left", 0, 185));
        InterpolatedAngleServo pivRight = new InterpolatedAngleServo(new SimpleServo(hardwareMap, "int_right", 0, 185));

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
