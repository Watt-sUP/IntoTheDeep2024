package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.CLAW_CLOSED;
import static org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem.CLAW_OPEN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.atomic.AtomicBoolean;

@Disabled
@TeleOp(name = "Claw Test", group = "Tests")
public class ClawTest extends CommandOpMode {
    @Override
    public void initialize() {
        AtomicBoolean clawOpen = new AtomicBoolean(false);

        SimpleServo clawServo = new SimpleServo(hardwareMap, "claw_servo", 0, 360);
        clawServo.setInverted(true);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamepadEx driver2 = new GamepadEx(gamepad2);

        driver2.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            clawOpen.set(!clawOpen.get());
        });

        schedule(
                new RunCommand(() -> {
                    if (clawOpen.get()) {
                        clawServo.turnToAngle(CLAW_OPEN);
                    } else {
                        clawServo.turnToAngle(CLAW_CLOSED);
                    }
                })
        );
    }
}
