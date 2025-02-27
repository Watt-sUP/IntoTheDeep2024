package org.firstinspires.ftc.teamcode.teleop.solo;

import static org.firstinspires.ftc.teamcode.teleop.TeleOpBase.SLIDES_ADJUST;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

import java.util.List;

@TeleOp(name = "TeleOp Luca", group = "Solo")
public class TeleOpSoloLuca extends CommandOpMode {
    public DriveSubsystem chassis;
    public IntakeSubsystem intake;
    public OuttakeSubsystem outtake;

    public Command intakeCollectCommand() {
        if (intake.getPivotState() == IntakeSubsystem.PivotState.COLLECT) {
            return new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.DOWN));
        } else if (intake.getPivotState() == IntakeSubsystem.PivotState.DOWN) {
            if (intake.getClawState() == IntakeSubsystem.ClawState.OPENED) {
                return new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED));
            } else {
                return new InstantCommand(() -> {
                    intake.setClawState(IntakeSubsystem.ClawState.OPENED);
                    intake.setPivotState(IntakeSubsystem.PivotState.COLLECT);
                });
            }
        }

        return new WaitCommand(0);
    }

    public Command hangCommand() {
        if (outtake.getSlidesState() == OuttakeSubsystem.SlidesState.HANG_PREPARE) {
            return new InstantCommand(() -> {
                outtake.setHanging(true);
                outtake.setSlidesState(OuttakeSubsystem.SlidesState.HANG);
            });
        }
        if (outtake.getSlidesState() == OuttakeSubsystem.SlidesState.HANG) {
            return new InstantCommand(() -> {
                outtake.setHanging(false);
                outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
            });
        }

        return new InstantCommand(() -> {
            outtake.setHanging(false);
            outtake.setSlidesState(OuttakeSubsystem.SlidesState.HANG_PREPARE);
        });
    }

    public Command basketCommand() {
        if (outtake.getArmState() != OuttakeSubsystem.ArmState.IN ||
                outtake.getPivotState() != OuttakeSubsystem.PivotState.IN ||
                outtake.getSlidesState() != OuttakeSubsystem.SlidesState.LOWERED) {
            return new InstantCommand(() -> {
                outtake.setArmState(OuttakeSubsystem.ArmState.IN);
                outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
            });
        } else {
            return new InstantCommand(() -> {
                outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                outtake.setPivotState(OuttakeSubsystem.PivotState.OUT);
                outtake.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET);
            });
        }
    }

    public Command specimenCommand() {
        if (outtake.getArmState() != OuttakeSubsystem.ArmState.SPECIMEN_COLLECT ||
                outtake.getPivotState() != OuttakeSubsystem.PivotState.SPECIMEN_COLLECT ||
                outtake.getSlidesState() != OuttakeSubsystem.SlidesState.LOWERED) {
            return new InstantCommand(() -> {
                outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN_COLLECT);
                outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT);
                outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
            });
        } else {
            return new InstantCommand(() -> {
                outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN);
                outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN_DEPOSIT);
                outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT);
            });
        }
    }

    @Override
    public void initialize() {
        this.reset();
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setMsTransmissionInterval(10);

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime loopTime = new ElapsedTime();

        chassis = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        TransferCommand transferCommand = new TransferCommand(intake, outtake);
        Trigger isTransferringTrigger = new Trigger(() -> !transferCommand.isScheduled());

        InstantCommand resetCommand = new InstantCommand(() -> {
            if (transferCommand.isScheduled()) {
                transferCommand.cancel();
            }

            intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);
            intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT);
            intake.setPivotState(IntakeSubsystem.PivotState.COLLECT);
            intake.setClawState(IntakeSubsystem.ClawState.OPENED);

            outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
            outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
            outtake.setArmState(OuttakeSubsystem.ArmState.IN);
            outtake.setClawState(OuttakeSubsystem.ClawState.OPENED);
        });

        GamepadEx driver = new GamepadEx(gamepad1);
        driver.gamepad.setLedColor(226 / 255f, 110 / 255f, 14 / 255f, Gamepad.LED_DURATION_CONTINUOUS);

        Trigger leftTrigger = new Trigger(() -> gamepad1.left_trigger >= 0.3);
        Trigger rightTrigger = new Trigger(() -> gamepad1.right_trigger >= 0.3);
        
        chassis.setMotorBrake(true);
        chassis.setAxes(driver::getLeftY, driver::getLeftX, driver::getRightX);

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> chassis.setMaxSpeed(0.5))
                .whenReleased(() -> chassis.setMaxSpeed(1));

        // Extendo
        rightTrigger
                .and(isTransferringTrigger)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            if (transferCommand.isScheduled())
                                transferCommand.cancel();

                            intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT);
                        }),
                        new WaitCommand(75),
                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.EXTENDING)),
                        new ConditionalCommand(
                                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT)),
                                () -> intake.getExtendoState() != IntakeSubsystem.ExtendoState.IN
                        ),
                        new WaitCommand(500),
                        new InstantCommand((() -> intake.setPivotState(IntakeSubsystem.PivotState.COLLECT)))
                ));

        // Collect
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .and(isTransferringTrigger)
                .whenActive(new SelectCommand(this::intakeCollectCommand));

        // Transfer
        driver.getGamepadButton(GamepadKeys.Button.Y)
                .toggleWhenPressed(transferCommand, true);

        // Slides
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .and(isTransferringTrigger)
                .whenActive(() -> outtake.adjustSlides(SLIDES_ADJUST));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .and(isTransferringTrigger)
                .whenActive(() -> outtake.adjustSlides(-SLIDES_ADJUST));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .and(isTransferringTrigger)
                .whenActive(new SelectCommand(this::basketCommand));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .and(isTransferringTrigger)
                .whenActive(new SelectCommand(this::specimenCommand));
        driver.getGamepadButton(GamepadKeys.Button.B)
                .and(isTransferringTrigger)
                .whenActive(new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED)));

        // Outtake Claw
        driver.getGamepadButton(GamepadKeys.Button.A)
                .and(isTransferringTrigger)
                .whenActive(outtake::toggleClaw);

        // Intake Rotate
        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .and(isTransferringTrigger)
                .and(new Trigger(() -> intake.getPivotState() != IntakeSubsystem.PivotState.UP))
                .whenActive(intake::nextRotation);
        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .and(isTransferringTrigger)
                .and(new Trigger(() -> intake.getPivotState() != IntakeSubsystem.PivotState.UP))
                .whenActive(intake::previousRotation);

        // Robot Reset
        driver.getGamepadButton(GamepadKeys.Button.BACK)
                .whenActive(resetCommand);

        // Hang
        driver.getGamepadButton(GamepadKeys.Button.X)
                .and(isTransferringTrigger)
                .whenActive(new SelectCommand(this::hangCommand));

        register(chassis, intake, outtake);

        schedule(
                new InstantCommand(() -> {
                    telemetry.addData("Status", "Initialized");
                    telemetry.update();
                }),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        resetCommand
                ),
                new RunCommand(() -> {
                    hubs.forEach(LynxModule::clearBulkCache);

                    telemetry.addData("Status", "Running");
                    telemetry.addData("Runtime", "%d seconds", (int) runtime.seconds());
                    telemetry.addData("Loop Time", loopTime.milliseconds());
                    telemetry.addLine();

                    telemetry.addData("Is Transferring", transferCommand.isScheduled() ? "Yes" : "No");
                    telemetry.addData("Is Hanging", outtake.isHanging() ? "Yes" : "No");
                    telemetry.addLine();

                    telemetry.addData("Intake Extendo State", intake.getExtendoState().toString());
                    telemetry.addData("Intake Pivot State", intake.getPivotState().toString());
                    telemetry.addData("Intake Claw State", intake.getClawState().toString());
                    telemetry.addData("Intake Rotate State", intake.getRotation().toString());
                    telemetry.addLine();

                    telemetry.addData("Outtake Claw State", outtake.getClawState().toString());
                    telemetry.addData("Outtake Arm State", outtake.getArmState().toString());
                    telemetry.addData("Outtake Pivot State", outtake.getPivotState().toString());
                    telemetry.addLine();

                    telemetry.addData("Outtake Slides State", outtake.getSlidesState().toString());
                    telemetry.addData("Outtake Slides Position", outtake.getSlidesPosition());
                    telemetry.addData("Outtake Slides Target", outtake.getSlidesTarget());
                    telemetry.addLine();

                    loopTime.reset();
                    telemetry.update();
                })
        );
    }
}
