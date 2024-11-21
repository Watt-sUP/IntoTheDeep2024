package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends CommandOpMode {
    @Override
    public void initialize() {
        this.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        ElapsedTime runtime = new ElapsedTime();

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        DriveSubsystem chassis = new DriveSubsystem(hardwareMap);

        chassis.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> chassis.setMaxSpeed(0.6))
                .whenReleased(() -> chassis.setMaxSpeed(1));

        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> chassis.setMaxSpeed(0.8))
                .whenReleased(() -> chassis.setMaxSpeed(1));

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(outtake::nextSlidesState);
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(outtake::previousSlidesState);

        driver2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(outtake::toggleArm);
        driver2.getGamepadButton(GamepadKeys.Button.A).whenPressed(outtake::toggleClaw);

        register(chassis, intake, outtake);

        schedule(
                new InstantCommand(() -> {
                    waitForStart();

                    hubs.forEach(LynxModule::clearBulkCache);
                    runtime.reset();

                    intake.retract();
                    
                    outtake.setArmState(OuttakeSubsystem.ArmState.IN);
                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                    outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                    outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED);
                }),
                new RunCommand(() -> {
                    telemetry.addData("Status", "Running");
                    telemetry.addData("Runtime", runtime.toString());
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
