package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.util.LABColor;

@Disabled
@TeleOp(name = "Intake Test", group = "Tests")
public class IntakeTest extends CommandOpMode {
    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        GamepadEx driver2 = new GamepadEx(gamepad2);

        driver2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(intake::startCollecting);

        driver2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(intake::stopCollecting);

        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(intake::startRemoving)
                .whenReleased(intake::stopRemoving);

        schedule(
                new RunCommand(() -> {
                    telemetry.addData("Sample Detected", intake.getSampleDetected().toString());
                    telemetry.addData("Is Collecting", intake.isCollecting());
                    telemetry.addLine();

                    LABColor color = intake.getColorDetected();
                    telemetry.addData("LAB", "%.3f, %.3f, %.3f", color.L, color.A, color.B);
                    telemetry.addData("Distance", "%.3f", intake.getSensorDistance(DistanceUnit.CM));
                    telemetry.addLine();

                    telemetry.addData("Distance", intake.getSensorDistance(DistanceUnit.CM));

                    telemetry.update();
                })
        );

        register(intake);
    }
}
