package robotcode.autonomous;

import static robotcode.autonomous.assets.AutonomousConstants.START_POSE_SPECIMEN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import robotcode.autonomous.assets.Observation;
import robotcode.autonomous.assets.SpikeSpecificSamples;
import robotcode.autonomous.assets.Submersible;
import robotcode.commands.FollowPathCommand;
import robotcode.pedroPathing.constants.FConstants;
import robotcode.pedroPathing.constants.LConstants;
import robotcode.subsystems.IntakeSubsystem;
import robotcode.subsystems.OuttakeSubsystem;
import robotcode.util.FixedSequentialCommandGroup;

@Autonomous(name = "Specimen Autonomous", group = "Autonomous")
public class AutoSpecimen extends CommandOpMode {
    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.setConstants(FConstants.class, LConstants.class);

        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE_SPECIMEN);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        register(intake, outtake);

        schedule(
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new RunCommand(() -> {
                            follower.update();

                            if (follower.isBusy()) {
                                follower.telemetryDebug(telemetry);
                            }
                        })
                ),
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),

                        new InstantCommand(() -> follower.setMaxPower(0.9)),

                        new InstantCommand(() -> {
                            intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);
                            intake.setPivotState(IntakeSubsystem.PivotState.UP);
                            intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT);
                            intake.setClawState(IntakeSubsystem.ClawState.OPENED);

                            outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED);

                            outtake._setSlidesPosition(20);
                            outtake._setArmPosition(0);
                            outtake._setPivotPosition(0);
                        }),

                        new InstantCommand(() -> {
                            outtake._setPivotPosition(OuttakeSubsystem.PIVOT_OUT);
                            outtake._setArmPosition(OuttakeSubsystem.ARM_OUT);
                        }),
                        new WaitCommand(180),

                        new FollowPathCommand(follower, Submersible.startPathSpecimen),

                        new InstantCommand(() -> outtake._setSlidesPosition(110)),
                        new InstantCommand(() -> outtake._setPivotPosition(OuttakeSubsystem.PIVOT_IN)),
                        new WaitCommand(350),

                        new FollowPathCommand(follower, Submersible.depositPath(0))
                                .setHoldEnd(false)
                                .withTimeout(500),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),

                        new FollowPathCommand(follower,
                                SpikeSpecificSamples.LEFT.getSubmersibleToSamplePath(
                                        Submersible.depositPose(0, true)
                                )
                        ).alongWith(
                                new InstantCommand(() -> {
                                    outtake._setSlidesPosition(0);
                                    outtake._setArmPosition(OuttakeSubsystem.ARM_SPECIMEN);
                                    outtake._setPivotPosition(OuttakeSubsystem.PIVOT_SPECIMEN_COLLECT);
                                })
                        ),

                        new FollowPathCommand(follower, SpikeSpecificSamples.LEFT.getPathToObservation())
                                .setHoldEnd(false),

                        new FollowPathCommand(follower, SpikeSpecificSamples.MIDDLE.getObservationToSamplePath(SpikeSpecificSamples.LEFT)),
                        new FollowPathCommand(follower, SpikeSpecificSamples.MIDDLE.getPathToObservation())
                                .setHoldEnd(false),
                        new FollowPathCommand(follower, SpikeSpecificSamples.MIDDLE.goIntoPrepareCollectPath()),

                        new InstantCommand(() -> follower.setMaxPower(0.6)),

                        new WaitCommand(150),

                        new FollowPathCommand(follower, Observation.collectPath)
                                .withTimeout(1350),

                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(250),
                        new InstantCommand(() -> {
                            outtake._setPivotPosition(OuttakeSubsystem.PIVOT_SPECIMEN_DEPOSIT);
                            outtake._setArmPosition(OuttakeSubsystem.ARM_OUT);
                            outtake._setSlidesPosition(20);
                        }),

                        new InstantCommand(() -> outtake._setSlidesPosition(20)),

                        new InstantCommand(() -> follower.setMaxPower(0.9)),

                        new FollowPathCommand(follower, Observation.toSubmersiblePath(1)),

                        new InstantCommand(() -> outtake._setSlidesPosition(110)),
                        new InstantCommand(() -> outtake._setPivotPosition(OuttakeSubsystem.PIVOT_IN)),
                        new WaitCommand(350),

                        new FollowPathCommand(follower, Submersible.depositPath(1)).setHoldEnd(false).withTimeout(750),
                        new InstantCommand(() -> {
                            outtake.setClawState(OuttakeSubsystem.ClawState.OPENED);
                        }),

                        new InstantCommand(() -> follower.setMaxPower(0.6)),

                        new FollowPathCommand(follower, Submersible.toCollectPath(1)).withTimeout(3500)
                                .alongWith(
                                        new InstantCommand(() -> {
                                            outtake._setSlidesPosition(0);
                                            outtake._setArmPosition(OuttakeSubsystem.ARM_SPECIMEN);
                                            outtake._setPivotPosition(OuttakeSubsystem.PIVOT_SPECIMEN_COLLECT);
                                        })
                                ),

                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),

                        new WaitCommand(250),
                        new InstantCommand(() -> {
                            outtake._setPivotPosition(OuttakeSubsystem.PIVOT_SPECIMEN_DEPOSIT);
                            outtake._setArmPosition(OuttakeSubsystem.ARM_OUT);
                        }),

                        new InstantCommand(() -> follower.setMaxPower(0.9)),

                        new InstantCommand(() -> outtake._setSlidesPosition(20)),

                        new FollowPathCommand(follower, Observation.toSubmersiblePath(2)),

                        new InstantCommand(() -> outtake._setSlidesPosition(110)),
                        new InstantCommand(() -> outtake._setPivotPosition(OuttakeSubsystem.PIVOT_IN)),
                        new WaitCommand(350),

                        new FollowPathCommand(follower, Submersible.depositPath(2)).setHoldEnd(false).withTimeout(750),
                        new InstantCommand(() -> {
                            outtake.setClawState(OuttakeSubsystem.ClawState.OPENED);
                        }),

                        new FollowPathCommand(follower, Submersible.toObservationParkPath(2))
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new InstantCommand(() -> {
                                                    intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);
                                                    intake.setPivotState(IntakeSubsystem.PivotState.DOWN);
                                                    intake.setClawState(IntakeSubsystem.ClawState.OPENED);
                                                }),
                                                new WaitCommand(500),
                                                new InstantCommand(() -> {
                                                    outtake._setSlidesPosition(0);

                                                    outtake._setArmPosition(OuttakeSubsystem.ARM_IN);
                                                    outtake._setPivotPosition(OuttakeSubsystem.PIVOT_IN);
                                                })
                                        )
                                )
                )
        );
    }
}
