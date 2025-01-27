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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class TeleOpBase extends CommandOpMode {
    public ElapsedTime runtime, loopTime;

    public GamepadEx driver1, driver2;
    public Trigger leftTrigger1, rightTrigger1, leftTrigger2, rightTrigger2;

    public AtomicBoolean isTransferring;
    public Trigger isTransferringTrigger;

    public DriveSubsystem chassis;
    public IntakeSubsystem intake;
    public OuttakeSubsystem outtake;

    @Override
    public void initialize() {
        this.reset();

        isTransferring = new AtomicBoolean(false);
        isTransferringTrigger = new Trigger(() -> !isTransferring.get());

        this.telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        runtime = new ElapsedTime();
        loopTime = new ElapsedTime();

        driver1 = new GamepadEx(gamepad1);
        driver2 = new GamepadEx(gamepad2);

        leftTrigger1 = new Trigger(() -> gamepad1.left_trigger >= 0.3);
        rightTrigger1 = new Trigger(() -> gamepad1.right_trigger >= 0.3);

        leftTrigger2 = new Trigger(() -> gamepad2.left_trigger >= 0.3);
        rightTrigger2 = new Trigger(() -> gamepad2.right_trigger >= 0.3);

        chassis = new DriveSubsystem(hardwareMap);

        chassis.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        /* Intake Commands */

        intake = new IntakeSubsystem(hardwareMap);

        /* Claw */
        leftTrigger2
                .and(isTransferringTrigger)
                .whenActive(intake::togglePivot);
        driver2.getGamepadButton(GamepadKeys.Button.A)
                .and(isTransferringTrigger)
                .whenActive(intake::toggleClaw);

        /* Claw Rotation */
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .and(isTransferringTrigger)
                .and(new Trigger(() -> intake.getPivotState() != IntakeSubsystem.PivotState.UP))
                .whenActive(intake::nextRotation);
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .and(isTransferringTrigger)
                .and(new Trigger(() -> intake.getPivotState() != IntakeSubsystem.PivotState.UP))
                .whenActive(intake::previousRotation);

        /* Extendo */
        rightTrigger1
                .and(isTransferringTrigger)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT)),
                        new WaitCommand(75),
                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.EXTENDING)),
                        new ConditionalCommand(
                                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT)),
                                () -> intake.getExtendoState() != IntakeSubsystem.ExtendoState.IN
                        )
                ));

        /* Outtake Commands */

        outtake = new OuttakeSubsystem(hardwareMap);

        /* Slides Direct */
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .and(isTransferringTrigger)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED)),
                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER)),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.IN))
                ));
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .and(isTransferringTrigger)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)),
                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.OUT)),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.OUT))
                ));

        /* Slides Progressive */
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .and(isTransferringTrigger)
                .whenActive(outtake::nextSlidesState);
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .and(isTransferringTrigger)
                .whenActive(outtake::previousSlidesState);

        /* Specimen collect and deposit */
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .and(isTransferringTrigger)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.OUT)),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT)),
                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN))
                ));
        rightTrigger2
                .and(isTransferringTrigger)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.DOWN)),
                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN)),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT)),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                ));

        /* Arm */
        driver2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .and(isTransferringTrigger)
                .whenActive(outtake::togglePivot);

        /* Transfer Command */
        
        SequentialCommandGroup transferCommand = new SequentialCommandGroup(
                new InstantCommand(() -> isTransferring.set(true)),
                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(250),
                new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED)),
                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER)),
                new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.IN)),
                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),
                new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT)),
                new WaitCommand(75),
                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                new WaitCommand(300),
                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
                new WaitCommand(150),
                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),
                new WaitCommand(100),
                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.OUT)),
                new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.OUT)),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.COLLECT)),
                new InstantCommand(() -> isTransferring.set(false))
        );

        driver2.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenActive(new ConditionalCommand(
                        transferCommand,
                        new SequentialCommandGroup(
                                new InstantCommand(() -> isTransferring.set(false)),
                                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.COLLECT)),
                                new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT)),
                                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),
                                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER)),
                                new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.IN)),
                                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                        ),
                        () -> !isTransferring.get()
                ));

        register(chassis, intake, outtake);

        schedule(
                new InstantCommand(() -> {
                    telemetry.addData("Status", "Initialized");
                    telemetry.update();
                }),
                new InstantCommand(() -> {
                    waitForStart();

                    runtime.reset();

                    intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);
                    intake.setClawState(IntakeSubsystem.ClawState.OPENED);
                    intake.setPivotState(IntakeSubsystem.PivotState.COLLECT);
                    intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT);

                    outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER);
                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                    outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                    outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED);
                }),
                new RunCommand(() -> hubs.forEach(LynxModule::clearBulkCache)),
                new RunCommand(() -> {
                    telemetry.addData("Status", "Running");
                    telemetry.addData("Runtime", "%.0f seconds", runtime.seconds());
                    telemetry.addData("Loop Time", loopTime.milliseconds());
                    telemetry.addLine();

                    telemetry.addData("Is Transferring", isTransferring.get());
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
                    telemetry.addData("Outtake Slides Current", "%.2f, %.2f", outtake.getSlidesCurrent()[0], outtake.getSlidesCurrent()[1]);
                    telemetry.addLine();

                    loopTime.reset();

                    telemetry.update();
                })
        );
    }
}
