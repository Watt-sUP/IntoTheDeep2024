package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name = "Intake Test", group = "Tests")
public class IntakeTest extends CommandOpMode {
    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        GamepadEx driver2 = new GamepadEx(gamepad2);

        TriggerReader leftTrigger = new TriggerReader(driver2, GamepadKeys.Trigger.LEFT_TRIGGER);
        TriggerReader rightTrigger = new TriggerReader(driver2, GamepadKeys.Trigger.RIGHT_TRIGGER);

        schedule(
                new RunCommand(() -> {
                    if (leftTrigger.isDown()) {
                        intake.startRemoving();
                    } else {
                        intake.stopRemoving();
                    }

                    if (rightTrigger.isDown()) {
                        intake.startCollecting();
                    } else {
                        intake.stopCollecting();
                    }
                }),
                new RunCommand(() -> {
                    telemetry.addData("Is Collecting", intake.isCollecting());
                    telemetry.addLine();

                    telemetry.update();
                })
        );

        register(intake);
    }
}
