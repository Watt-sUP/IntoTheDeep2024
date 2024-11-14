package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Slides Tuner", group = "Tuners")
public class SlidesTuner extends LinearOpMode {
    public static int POSITION = 0;
    public static double MAX_SPEED = 1, kP = 0;

    @Override
    public void runOpMode() {
        Motor slidesMotor = new Motor(hardwareMap, "slides");
        slidesMotor.setRunMode(Motor.RunMode.PositionControl);
        slidesMotor.setPositionCoefficient(kP);
        slidesMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {
            slidesMotor.setTargetPosition(POSITION);
            slidesMotor.setPositionCoefficient(kP);

            if (!slidesMotor.atTargetPosition()) {
                slidesMotor.set(MAX_SPEED);
            }
        }
    }
}
