package org.firstinspires.ftc.teamcode.autonomous;

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

@Autonomous(name = "Specimen Autonomous", group = "Autonomous")
public class SpecimenAuto extends AutonomousOpMode {
    @Override
    public void initialize() {
        super.initialize();
        startSpecimen();
        enableInit();

        schedule(
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new InstantCommand(() -> {
                            intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);

                            outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                            outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT);
                            outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN);
                        }),

                        new WaitCommand(250),

                        new FollowPointCommand(follower, Submersible.depositPose(0, false), 1)
                                .andThen(new WaitCommand(75)),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                                .andThen(new WaitCommand(100)),

                        new FollowPathCommand(follower, Observation.samplesToObservationPath).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new InstantCommand(() -> {
                                            outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                                            outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN);
                                            outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT);
                                        })
                                )
                        ),

                        new FollowPointCommand(follower, Observation.prepareCollectPose(0), 7),
                        new InstantCommand(() -> follower.setMaxPower(0.55)),
                        new FollowPointCommand(follower, Observation.collectPose(0), 0.1)
                                .withTimeout(650),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED))
                                .andThen(new WaitCommand(200)),

                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN))
                                .andThen(new WaitCommand(100)),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        new FollowPointCommand(follower, Submersible.depositPose(1, true), 8)
                                .alongWith(
                                        new InstantCommand(() -> {
                                            outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                                            outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT);
                                        })
                                ),
                        new FollowPointCommand(follower, Submersible.depositPose(1, false), 1)
                                .andThen(new WaitCommand(75)),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                                .andThen(new WaitCommand(100)),

                        new FollowPointCommand(follower, Submersible.depositPose(1, true), 10)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(250),
                                                new InstantCommand(() -> {
                                                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                                                    outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN);
                                                    outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT);
                                                })
                                        )
                                ),
                        new FollowPointCommand(follower, Observation.prepareCollectPose(1), 7),
                        new InstantCommand(() -> follower.setMaxPower(0.565)),
                        new FollowPointCommand(follower, Observation.collectPose(1), 0.1)
                                .withTimeout(650),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED))
                                .andThen(new WaitCommand(200)),

                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN))
                                .andThen(new WaitCommand(100)),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        new FollowPointCommand(follower, Submersible.depositPose(2, true), 8)
                                .alongWith(
                                        new InstantCommand(() -> {
                                            outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                                            outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT);
                                        })
                                ),
                        new FollowPointCommand(follower, Submersible.depositPose(2, false), 1)
                                .andThen(new WaitCommand(75)),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                                .andThen(new WaitCommand(100)),

                        new FollowPointCommand(follower, Submersible.depositPose(2, true), 10)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(250),
                                                new InstantCommand(() -> {
                                                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                                                    outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN);
                                                    outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT);
                                                })
                                        )
                                ),
                        new FollowPointCommand(follower, Observation.prepareCollectPose(2), 7),
                        new InstantCommand(() -> follower.setMaxPower(0.565)),
                        new FollowPointCommand(follower, Observation.collectPose(2), 0.1)
                                .withTimeout(650),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED))
                                .andThen(new WaitCommand(200)),

                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN))
                                .andThen(new WaitCommand(100)),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        new FollowPointCommand(follower, Submersible.depositPose(3, true), 8)
                                .alongWith(
                                        new InstantCommand(() -> {
                                            outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                                            outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT);
                                        })
                                ),
                        new FollowPointCommand(follower, Submersible.depositPose(3, false), 1)
                                .andThen(new WaitCommand(75)),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                                .andThen(new WaitCommand(100)),

                        new FollowPointCommand(follower, Submersible.depositPose(3, true), 10)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(250),
                                                new InstantCommand(() -> {
                                                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                                                    outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN);
                                                    outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT);
                                                })
                                        )
                                ),
                        new FollowPointCommand(follower, Observation.prepareCollectPose(3), 7),
                        new InstantCommand(() -> follower.setMaxPower(0.565)),
                        new FollowPointCommand(follower, Observation.collectPose(3), 0.1)
                                .withTimeout(650),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED))
                                .andThen(new WaitCommand(200)),

                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN))
                                .andThen(new WaitCommand(100)),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        new FollowPointCommand(follower, Submersible.depositPose(4, true), 8)
                                .alongWith(
                                        new InstantCommand(() -> {
                                            outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                                            outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT);
                                        })
                                ),
                        new FollowPointCommand(follower, Submersible.depositPose(4, false), 1)
                                .andThen(new WaitCommand(75)),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                                .andThen(new WaitCommand(100)),

                        new FollowPointCommand(follower, Observation.parkPose)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(250),
                                                new InstantCommand(() -> {
                                                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                                                    intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT);
                                                })
                                        )
                                )
                                .andThen(new InstantCommand(() -> follower.breakFollowing()))
                )
        );
    }
}
