package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Slides Tuner", group = "Tuners")
public class SlidesTuner extends LinearOpMode {
    public static int POSITION = 0;

    @Override
    public void runOpMode() {
        DcMotor slidesMotor = hardwareMap.dcMotor.get("slides");
        slidesMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor.setTargetPosition(0);
        slidesMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        slidesMotor.setPower(1);

        while (opModeIsActive()) {
            slidesMotor.setTargetPosition(POSITION);
        }
    }
}
