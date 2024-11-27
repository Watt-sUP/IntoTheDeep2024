package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp (Luca + Miron)", group = "TeleOp")
public class TeleOpLucaMiron extends CommandOpMode {
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

        TriggerReader leftTrigger2 = new TriggerReader(driver2, GamepadKeys.Trigger.LEFT_TRIGGER, 0.3);
        TriggerReader rightTrigger2 = new TriggerReader(driver2, GamepadKeys.Trigger.RIGHT_TRIGGER, 0.3);

        DriveSubsystem chassis = new DriveSubsystem(hardwareMap);

        chassis.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> chassis.setMaxSpeed(0.4))
                .whenReleased(() -> chassis.setMaxSpeed(1));

        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> chassis.setMaxSpeed(0.6))
                .whenReleased(() -> chassis.setMaxSpeed(1));

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        driver2.getGamepadButton(GamepadKeys.Button.A).whenPressed(intake::toggleClaw);

        driver2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(intake::previousRotation);
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(intake::nextRotation);

        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(outtake::nextSlidesState);
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(outtake::previousSlidesState);

        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(outtake::togglePivot);

        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED));
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET));

        driver2.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(outtake::toggleArm),
                        new InstantCommand(outtake::togglePivot)
                )
        );
        driver2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(outtake::toggleClaw);

        driver2.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER)),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.IN)),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),
                        new WaitCommand(350),
                        new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT)),
                        new WaitCommand(650),
                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.IN)),
                        new WaitCommand(450),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),
                        new WaitCommand(350),
                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.COLLECT))
                )
        );

        register(chassis, intake, outtake);

        schedule(
                new InstantCommand(() -> {
                    waitForStart();

                    runtime.reset();

                    intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);
                    intake.setClawState(IntakeSubsystem.ClawState.OPENED);
                    intake.setPivotState(IntakeSubsystem.PivotState.COLLECT);
                    intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT);

                    outtake.setArmState(OuttakeSubsystem.ArmState.IN);
                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                    outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                    outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED);
                }),
                new RunCommand(() -> {
                    hubs.forEach(LynxModule::clearBulkCache);

                    if (leftTrigger2.wasJustPressed()) {
                        intake.togglePivot();
                    }
                    if (rightTrigger2.wasJustPressed()) {
                        intake.toggleExtendo();
                    }

                    leftTrigger2.readValue();
                    rightTrigger2.readValue();
                }),
                new RunCommand(() -> {
                    telemetry.addData("Status", "Running");
                    telemetry.addData("Runtime", runtime.seconds());
                    telemetry.addLine();

                    telemetry.addData("Intake Extendo State", intake.getExtendoState().toString());
                    telemetry.addData("Intake Pivot State", intake.getPivotState().toString());
                    telemetry.addData("Intake Claw State", intake.getClawState().toString());
                    telemetry.addData("Intake Rotate State", intake.getRotation().toString());
                    telemetry.addLine();

                    telemetry.addData("Outtake Claw State", outtake.getClawState().toString());
                    telemetry.addData("Outtake Arm State", outtake.getArmState().toString());
                    telemetry.addData("Outtake Pivot State", outtake.getPivotState().toString());
                    telemetry.addData("Outtake Slides State", outtake.getSlidesState().toString());
                    telemetry.addLine();

                    telemetry.update();
                })
        );
    }
}
