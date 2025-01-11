package robotcode.autonomous.old;

import static robotcode.autonomous.assets.AutonomousConstants.START_POSE_SPECIMEN;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import robotcode.autonomous.assets.Observation;
import robotcode.autonomous.assets.SpikeSpecificSamples;
import robotcode.autonomous.assets.Submersible;
import robotcode.commands.FollowPathCommand;
import robotcode.pedroPathing.follower.Follower;
import robotcode.subsystems.IntakeSubsystem;
import robotcode.subsystems.OuttakeSubsystem;
import robotcode.util.FixedSequentialCommandGroup;

@Disabled
@Autonomous(name = "2 Specimen Autonomous", group = "Old Auto")
public class TwoSpecimenAuto extends CommandOpMode {
    @Override
    public void initialize() {
        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE_SPECIMEN);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        register(intake, outtake);

        schedule(
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new RunCommand(follower::update),
                        new RunCommand(() -> follower.telemetryDebug(telemetry))
                ),
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT)),
                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.OUT)),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT)),
                        new InstantCommand(() -> follower.setMaxPower(0.6)),
                        new FollowPathCommand(follower, Submersible.startPathSpecimen).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(350),
                                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN)),
                                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.OUT))
                                )
                        ).withTimeout(5000),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT)),
                        new WaitCommand(450),
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPathCommand(follower, Submersible.depositPath(0)).setHoldEnd(false).withTimeout(3000),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),
                        new InstantCommand(() -> follower.setMaxPower(0.6)),
                        new FollowPathCommand(follower,
                                SpikeSpecificSamples.LEFT.getSubmersibleToSamplePath(
                                        Submersible.depositPose(0, true)
                                )
                        ).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED)),
                                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
                                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.IN))
                                )
                        ),
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPathCommand(follower, SpikeSpecificSamples.LEFT.getPathToObservation()).setHoldEnd(false),
                        new InstantCommand(() -> follower.setMaxPower(0.6)),
                        new FollowPathCommand(follower, SpikeSpecificSamples.MIDDLE.getObservationToSamplePath(SpikeSpecificSamples.LEFT)).setHoldEnd(false),
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPathCommand(follower, SpikeSpecificSamples.MIDDLE.getPathToObservation()).setHoldEnd(false),
                        new FollowPathCommand(follower, SpikeSpecificSamples.MIDDLE.goIntoPrepareCollectPath()).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),
                                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.DOWN)),
                                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN)),
                                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT)),
                                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                                )
                        ),
                        new InstantCommand(() -> follower.setMaxPower(0.55)),
                        new FollowPathCommand(follower, Observation.collectPath).setHoldEnd(false).withTimeout(2500),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(250),
                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.OUT)),
                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN)),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.OUT)),
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPathCommand(follower, Observation.toSubmersiblePath(1)).withTimeout(4500),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT)),
                        new WaitCommand(450),
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPathCommand(follower, Submersible.depositPath(1)).setHoldEnd(false).withTimeout(3000),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),
                        new FollowPathCommand(follower, Submersible.toObservationParkPath(1)).alongWith(new SequentialCommandGroup(
                                new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED)),
                                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
                                new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.IN))
                        ))
                )
        );
    }
}
