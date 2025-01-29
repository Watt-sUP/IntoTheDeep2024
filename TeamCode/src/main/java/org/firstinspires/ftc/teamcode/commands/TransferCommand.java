package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
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
        addCommands(
                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(250),
                new InstantCommand(() -> {
                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                    outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER);
                    outtake.setClawState(OuttakeSubsystem.ClawState.OPENED);
                    outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                    intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT);
                }),
                new WaitCommand(75),
                new InstantCommand(() -> {
                    intake.setPivotState(IntakeSubsystem.PivotState.UP);
                    intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);
                }),
                new WaitCommand(375),
                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
                new WaitCommand(150),
                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),
                new WaitCommand(100),
                new InstantCommand(() -> {
                    outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                    outtake.setPivotState(OuttakeSubsystem.PivotState.OUT);
                }),
                new WaitCommand(100),
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

            outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER);
            outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
            outtake.setClawState(OuttakeSubsystem.ClawState.OPENED);
        }

        super.end(interrupted);
    }
}
