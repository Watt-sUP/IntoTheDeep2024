package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

import java.util.List;

@TeleOp(name = "Solo Drive")
public class TeleOpSolo extends CommandOpMode {

    public ElapsedTime clawTimer, hangTimer;

    public GamepadEx driver;

    public Trigger leftTrigger, rightTrigger;
    public Trigger isTransferringTrigger;

    public DriveSubsystem chassis;
    public IntakeSubsystem intake;
    public OuttakeSubsystem outtake;

    @Override
    public void initialize() {
        this.reset();
        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        chassis = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        driver = new GamepadEx(gamepad1);

        clawTimer = new ElapsedTime();
        hangTimer = new ElapsedTime();

        leftTrigger = new Trigger(() -> gamepad1.left_trigger >= 0.3);
        rightTrigger = new Trigger(() -> gamepad1.right_trigger >= 0.3);

        TransferCommand transferCommand = new TransferCommand(intake, outtake);
        isTransferringTrigger = new Trigger(() -> !transferCommand.isScheduled());

        chassis.setTeleOpMode(true);
        chassis.setTeleOpAxes(() -> -driver.getLeftY(), driver::getLeftX, driver::getRightX);

        leftTrigger
                .and(isTransferringTrigger)
                .whenActive(intake::togglePivot);

        rightTrigger
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

        driver.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> intake.getPivotState() == IntakeSubsystem.PivotState.DOWN))
                .toggleWhenActive(transferCommand, true);

        driver.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> outtake.getArmState() == OuttakeSubsystem.ArmState.OUT))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(outtake::toggleClaw),
                        new WaitCommand(500),
                        new InstantCommand(() -> {
                            outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                            outtake.setArmState(OuttakeSubsystem.ArmState.IN);
                            outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                        })
                ));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> outtake.getArmState() == OuttakeSubsystem.ArmState.SPECIMEN_DEPOSIT))
                .and(new Trigger(() -> clawTimer.milliseconds() >= 500))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(outtake::toggleClaw),
                        new WaitCommand(750),
                        new InstantCommand(() -> {
                            outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                            outtake.setArmState(OuttakeSubsystem.ArmState.IN);
                            outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                            clawTimer.reset();
                        })
                ));

        driver.getGamepadButton(GamepadKeys.Button.A)
                .and(new Trigger(() -> outtake.getArmState() == OuttakeSubsystem.ArmState.SPECIMEN_COLLECT))
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(outtake::toggleClaw),
                        new WaitCommand(250),
                        new InstantCommand(() -> {
                            clawTimer.reset();
                            outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT);
                            outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN_DEPOSIT);
                            outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN);
                        })
                ));

        driver.getGamepadButton(GamepadKeys.Button.X)
                .whenActive(() -> {
                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                    outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN_COLLECT);
                    outtake.setClawState(OuttakeSubsystem.ClawState.OPENED);
                    outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT);
                });

        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .and(new Trigger(() -> intake.getPivotState() != IntakeSubsystem.PivotState.UP))
                .whenActive(intake::previousRotation);

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .and(new Trigger(() -> intake.getPivotState() != IntakeSubsystem.PivotState.UP))
                .whenActive(intake::nextRotation);

        driver.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .and(new Trigger(() -> outtake.getSlidesState() != OuttakeSubsystem.SlidesState.HIGH_BASKET))
                .whenActive(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET));

        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .and(new Trigger(() -> outtake.getSlidesState() != OuttakeSubsystem.SlidesState.LOW_BASKET))
                .whenActive(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOW_BASKET));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenActive(() -> outtake._setSlidesPosition(outtake.getSlidesPosition() + 200));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenActive(() -> outtake._setSlidesPosition(outtake.getSlidesPosition() - 200));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenActive(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED));

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .and(new Trigger(() -> outtake.getSlidesState() == OuttakeSubsystem.SlidesState.HANG_PREPARE))
                .and(new Trigger(() -> hangTimer.milliseconds() <= 500))
                .whenActive(() -> {
                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.HANG);
                    hangTimer.reset();
                });

        driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .and(new Trigger(() -> outtake.getSlidesState() != OuttakeSubsystem.SlidesState.HANG_PREPARE))
                .whenActive(() -> {
                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.HANG_PREPARE);
                    hangTimer.reset();
                });

        if (outtake.getSlidesState() == OuttakeSubsystem.SlidesState.HIGH_BASKET) {
            chassis.setMaxSpeed(0.8);
        }
        else {
            chassis.setMaxSpeed(1);
        }

        register(chassis, intake, outtake);

        schedule(
                new InstantCommand(() -> {
                    telemetry.addData("Status", "Initialized");
                    telemetry.update();
                }),
                new InstantCommand(() -> {
                    waitForStart();

                    intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);
                    intake.setClawState(IntakeSubsystem.ClawState.OPENED);
                    intake.setPivotState(IntakeSubsystem.PivotState.COLLECT);
                    intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT);

                    outtake.setArmState(OuttakeSubsystem.ArmState.IN);
                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                    outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                    outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED);
                }),
                new RunCommand(() -> hubs.forEach(LynxModule::clearBulkCache)),
                new RunCommand(() -> {
                    telemetry.addData("I love PETRICA", "true");
                    telemetry.addLine();

                    telemetry.addData("Status", "Running");
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

                    telemetry.update();
                })
        );
    }
}

// Extendo: right trigger
// Intake Pivot: Left trigger
// Intake claw toggle (ONLY IF PIVOT DOWN): X
// WHILE TRANSFER: X = CANCEL
// SQUARE: OUTTAKE ROTATION
// DPAD: GLISIERE
// LEFT DPAD: HIGH BASKET
// RIGHT DPAD: LOW BASKET
// RIGHT BUMBER: LOWER FULLY
// INTAKE SPIN: LEFT JOYSTICK + RIGHT JOYSTICK
