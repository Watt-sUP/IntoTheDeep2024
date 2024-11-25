package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Disabled
@TeleOp(name = "Motor Test", group = "Tests")
public class MotorTest extends CommandOpMode {
    public static String MOTOR_NAME = "test";
    public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static DcMotor.ZeroPowerBehavior ZERO_POWER_BEHAVIOR = DcMotor.ZeroPowerBehavior.FLOAT;

    @Override
    public void initialize() {
        GamepadEx driver2 = new GamepadEx(gamepad2);

        DcMotor motor = hardwareMap.dcMotor.get(MOTOR_NAME);
        motor.setDirection(DIRECTION);
        motor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);

        schedule(
                new RunCommand(() -> {
                    motor.setDirection(DIRECTION);
                    motor.setZeroPowerBehavior(ZERO_POWER_BEHAVIOR);

                    motor.setPower(driver2.getLeftY());
                })
        );
    }
}
