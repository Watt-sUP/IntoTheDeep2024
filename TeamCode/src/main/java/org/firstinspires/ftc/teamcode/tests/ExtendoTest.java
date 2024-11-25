package org.firstinspires.ftc.teamcode.tests;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.InterpolatedPositionServo;

@Config
@TeleOp(name = "Extendo Test", group = "Tests")
public class ExtendoTest extends LinearOpMode {
    public static double POSITION = 0.0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        InterpolatedPositionServo extLeft = new InterpolatedPositionServo(new SimpleServo(hardwareMap, "ext_left", 0, 220));
        InterpolatedPositionServo extRight = new InterpolatedPositionServo(new SimpleServo(hardwareMap, "ext_right", 0, 220));

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

        waitForStart();

        while (opModeIsActive()) {
            extLeft.setToPosition(POSITION);
            extRight.setToPosition(POSITION);

            telemetry.addData("Ext Left Position", extLeft.getCurrentPosition());
            telemetry.addData("Ext Right Position", extRight.getCurrentPosition());
            telemetry.update();
        }
    }
}
