package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Motor Tuner", group = "Tuners")
public class MotorTuner extends LinearOpMode {
    public static String name = "test";
    public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = (DcMotorEx) hardwareMap.dcMotor.get(name);

        GamepadEx driver1 = new GamepadEx(gamepad1);

        motor.setDirection(DIRECTION);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(driver1.getLeftY());
            motor.setDirection(DIRECTION);

            telemetry.addData("Motor Position", motor.getCurrentPosition());
            telemetry.addData("Motor Target", motor.getTargetPosition());
            telemetry.update();
        }
    }
}
