package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@TeleOp(name = "Outtake Test", group = "Tests")
public class OuttakeTest extends CommandOpMode {
    @Override
    public void initialize() {
        this.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamepadEx driver2 = new GamepadEx(gamepad2);

        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(outtake::nextSlidesState);
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(outtake::previousSlidesState);

        driver2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(outtake::toggleArm);
        driver2.getGamepadButton(GamepadKeys.Button.X).whenPressed(outtake::togglePivot);

        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED));
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET));

        driver2.getGamepadButton(GamepadKeys.Button.A).whenPressed(outtake::toggleClaw);

        register(outtake);

        schedule(
                new RunCommand(() -> {
                    telemetry.addData("Status", "Running");
                    telemetry.addLine();

                    telemetry.addData("Claw", outtake.getClawState().toString());
                    telemetry.addData("Arm", outtake.getArmState().toString());
                    telemetry.addData("Pivot", outtake.getPivotState().toString());
                    telemetry.addData("Slides", outtake.getSlidesState().toString());
                    telemetry.addLine();

                    telemetry.update();
                })
        );
    }
}
