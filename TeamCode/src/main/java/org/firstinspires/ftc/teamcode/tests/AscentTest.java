package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.AscentSubsystem;

@Config
@TeleOp(name = "Ascent Test", group = "Tests")
public class AscentTest extends LinearOpMode {
    public static double ANGLE = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        AscentSubsystem ascent = new AscentSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            ascent.setAngle(ANGLE);

            ascent.periodic();

            telemetry.addData("Angle", ascent.getAngle());
            telemetry.addData("State", ascent.getState());

            telemetry.update();
        }
    }
}
