package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.autonomous.assets.Observation;
import org.firstinspires.ftc.teamcode.autonomous.assets.Submersible;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.FollowPointCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

import java.util.concurrent.atomic.AtomicInteger;

@Autonomous(name = "Specimen Autonomous", group = "Autonomous")
public class SpecimenAuto extends AutonomousOpMode {
    private final AtomicInteger currentSpecimen = new AtomicInteger(0);

    public Command collectAndDeposit() {
        int pos = currentSpecimen.incrementAndGet();

        return new FixedSequentialCommandGroup(
                new FollowPointCommand(follower, Observation.prepareCollectPose(pos - 1), 10)
                        .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new InstantCommand(() -> {
                                            outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                                            outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN_COLLECT);
                                            outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT);
                                        })
                                )
                        ),

                new InstantCommand(() -> follower.setMaxPower(0.55)),
                new FollowPointCommand(follower, Observation.collectPose(pos - 1), 0.1)
                        .withTimeout(700),
                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED))
                        .andThen(new WaitCommand(175)),

                new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN))
                        .andThen(new WaitCommand(100)),

                new InstantCommand(() -> follower.setMaxPower(1)),

                new FollowPointCommand(follower, Submersible.depositPose(pos, true), 8)
                        .alongWith(
                                new InstantCommand(() -> {
                                    outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN_DEPOSIT);
                                    outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT);
                                })
                        ),

                new FollowPointCommand(follower, Submersible.depositPose(pos, false), 1)
                        .andThen(new WaitCommand(75)),
                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                        .andThen(new WaitCommand(125)),

                new FollowPointCommand(follower, Submersible.depositPose(pos, true), 12)
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        startSpecimen();
        enableInit();
        enableLimelight();

        schedule(
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new InstantCommand(() -> {
                            intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);

                            outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN_DEPOSIT);
                            outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT);
                            outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN);
                        }),

                        new WaitCommand(250),

                        // Preload Deposit

                        new FollowPointCommand(follower, Submersible.depositPose(0, false), 1)
                                .andThen(new WaitCommand(25)),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                                .andThen(new WaitCommand(100)),

                        // Spike mark samples to Observation

                        new FollowPathCommand(follower, Observation.samplesToObservationPath).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new InstantCommand(() -> {
                                            outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                                            outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN_COLLECT);
                                            outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT);
                                        })
                                )
                        ),

                        collectAndDeposit(),
                        collectAndDeposit(),
                        collectAndDeposit(),
                        collectAndDeposit(),

                        // Park

                        new FollowPointCommand(follower, Observation.parkPose)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(700),
                                                new InstantCommand(() -> {
                                                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                                                    intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT);
                                                })
                                        )
                                )
                )
        );
    }
}
