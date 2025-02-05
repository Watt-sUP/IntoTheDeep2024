package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

public class TransferCommand extends FixedSequentialCommandGroup {
    private final IntakeSubsystem intake;
    private final OuttakeSubsystem outtake;

    public TransferCommand(IntakeSubsystem intakeSubsystem, OuttakeSubsystem outtakeSubsystem) {
        intake = intakeSubsystem;
        outtake = outtakeSubsystem;

        addRequirements(intake, outtake);

        if (intake.getClawState() != IntakeSubsystem.ClawState.CLOSED) {
            addCommands(
                    new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED))
                            .andThen(new WaitCommand(250))
            );
        }

        ParallelCommandGroup initCommand = new ParallelCommandGroup();

        if (outtake.getSlidesState() != OuttakeSubsystem.SlidesState.LOWERED) {
            initCommand.addCommands(
                    new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED))
                            .andThen(new WaitCommand(150))
            );
        }

        if (outtake.getArmState() != OuttakeSubsystem.ArmState.IN || outtakeSubsystem.getPivotState() != OuttakeSubsystem.PivotState.IN) {
            initCommand.addCommands(
                    new InstantCommand(() -> {
                        outtake.setArmState(OuttakeSubsystem.ArmState.IN);
                        outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                    }).andThen(new WaitCommand(250))
            );
        }

        if (outtake.getClawState() != OuttakeSubsystem.ClawState.OPENED) {
            initCommand.addCommands(
                    new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                            .andThen(new WaitCommand(150))
            );
        }

        if (intake.getRotation() != IntakeSubsystem.RotationState.STRAIGHT) {
            initCommand.addCommands(
                    new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT))
                            .andThen(new WaitCommand(75))
            );
        }

        addCommands(initCommand);

        if (intake.getPivotState() != IntakeSubsystem.PivotState.UP || intake.getExtendoState() != IntakeSubsystem.ExtendoState.IN) {
            addCommands(
                    new InstantCommand(() -> {
                        intake.setPivotState(IntakeSubsystem.PivotState.UP);
                        intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);
                    }).andThen(new WaitCommand(450))
            );
        }

        addCommands(
                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER))
                        .andThen(new WaitCommand(150)),
                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED))
                        .andThen(new WaitCommand(100)),
                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED))
                        .andThen(new WaitCommand(100)),
                new InstantCommand(() -> {
                    outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                    outtake.setPivotState(OuttakeSubsystem.PivotState.OUT);
                }).andThen(new WaitCommand(100)),
                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.COLLECT))
        );
    }

    @Override
    public void end(boolean interrupted) {
        if (finished)
            return;

        if (interrupted) {
            intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);
            intake.setPivotState(IntakeSubsystem.PivotState.COLLECT);
            intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT);
            intake.setClawState(IntakeSubsystem.ClawState.OPENED);

            outtake.setArmState(OuttakeSubsystem.ArmState.IN);
            outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
            outtake.setClawState(OuttakeSubsystem.ClawState.OPENED);
        }

        super.end(interrupted);
    }
}
