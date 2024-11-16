package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
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

        TriggerReader leftTrigger = new TriggerReader(driver2, GamepadKeys.Trigger.LEFT_TRIGGER, 0.3);
        TriggerReader rightTrigger = new TriggerReader(driver2, GamepadKeys.Trigger.RIGHT_TRIGGER, 0.3);

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

        driver2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(outtake::toggleArm),
                        new WaitCommand(250),
                        new ConditionalCommand(
                                new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.OUT)),
                                new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.IN)),
                                () -> outtake.getArmState() == OuttakeSubsystem.ArmState.OUT
                        )
                )
        );
        driver2.getGamepadButton(GamepadKeys.Button.X).whenPressed(outtake::togglePivot);

        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED));
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET));

        driver2.getGamepadButton(GamepadKeys.Button.A).whenPressed(outtake::toggleClaw);

        register(chassis, intake, outtake);

        schedule(
                new InstantCommand(() -> hubs.forEach(LynxModule::clearBulkCache)),
                new InstantCommand(runtime::reset),
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
                    telemetry.addData("Status", "Running");
                    telemetry.addData("Runtime", runtime.toString());
                    telemetry.addLine();

                    telemetry.addData("Is Collecting", intake.isCollecting());
                    telemetry.addData("Is Removing", intake.isRemoving());
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
