package org.firstinspires.ftc.teamcode.autonomous.experiments;

import static org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousConstants.START_POSE_SPECIMEN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.assets.Observation;
import org.firstinspires.ftc.teamcode.autonomous.assets.SpikeSpecificSamples;
import org.firstinspires.ftc.teamcode.autonomous.assets.Submersible;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

@Autonomous(name = "4 Specimen Autonomous", group = "Auto Experiments")
public class FourAutoSpecimen extends CommandOpMode {
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
                        new WaitCommand(250),

                        new InstantCommand(() -> follower.setMaxPower(0.6)),

                        new FollowPathCommand(follower, Submersible.startPathSpecimen),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        new InstantCommand(() -> outtake._setSlidesPosition(110)),
                        new InstantCommand(() -> outtake._setPivotPosition(OuttakeSubsystem.PIVOT_IN)),
                        new WaitCommand(350),

                        new FollowPathCommand(follower, Submersible.depositPath(0))
                                .setHoldEnd(false)
                                .withTimeout(600),
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
                        new FollowPathCommand(follower, SpikeSpecificSamples.RIGHT.getObservationToSamplePath(SpikeSpecificSamples.MIDDLE))
                                .withTimeout(1250),
                        new FollowPathCommand(follower, SpikeSpecificSamples.RIGHT.getPathToObservation()).withTimeout(1400),
                        new FollowPathCommand(follower, SpikeSpecificSamples.RIGHT.goIntoPrepareCollectPath()),

                        new InstantCommand(() -> follower.setMaxPower(0.9)),

                        new FollowPathCommand(follower, Observation.collectPath)
                                .withTimeout(800),

                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(250),
                        new InstantCommand(() -> {
                            outtake._setPivotPosition(OuttakeSubsystem.PIVOT_SPECIMEN_DEPOSIT);
                            outtake._setArmPosition(OuttakeSubsystem.ARM_OUT);
                            outtake._setSlidesPosition(20);
                        }),

                        new InstantCommand(() -> outtake._setSlidesPosition(20)),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        new FollowPathCommand(follower, Observation.toSubmersiblePath(1)),

                        new InstantCommand(() -> outtake._setSlidesPosition(110)),
                        new InstantCommand(() -> outtake._setPivotPosition(OuttakeSubsystem.PIVOT_IN)),
                        new WaitCommand(200),

                        new FollowPathCommand(follower, Submersible.depositPath(1)).setHoldEnd(false).withTimeout(600),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),

                        new InstantCommand(() -> follower.setMaxPower(0.92)),

                        new FollowPathCommand(follower, Submersible.toCollectPath(1)).withTimeout(3000)
                                .alongWith(
                                        new InstantCommand(() -> {
                                            outtake._setSlidesPosition(0);
                                            outtake._setArmPosition(OuttakeSubsystem.ARM_SPECIMEN);
                                            outtake._setPivotPosition(OuttakeSubsystem.PIVOT_SPECIMEN_COLLECT);
                                        })
                                ),

                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),

                        new WaitCommand(175),
                        new InstantCommand(() -> {
                            outtake._setPivotPosition(OuttakeSubsystem.PIVOT_SPECIMEN_DEPOSIT);
                            outtake._setArmPosition(OuttakeSubsystem.ARM_OUT);
                        }),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        new InstantCommand(() -> outtake._setSlidesPosition(20)),

                        new FollowPathCommand(follower, Observation.toSubmersiblePath(2)),

                        new InstantCommand(() -> outtake._setSlidesPosition(110)),
                        new InstantCommand(() -> outtake._setPivotPosition(OuttakeSubsystem.PIVOT_IN)),
                        new WaitCommand(200),

                        new FollowPathCommand(follower, Submersible.depositPath(2)).setHoldEnd(false).withTimeout(600),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),

                        new InstantCommand(() -> follower.setMaxPower(0.92)),

                        new FollowPathCommand(follower, Submersible.toCollectPath(2)).withTimeout(3000)
                                .alongWith(
                                        new InstantCommand(() -> {
                                            outtake._setSlidesPosition(0);
                                            outtake._setArmPosition(OuttakeSubsystem.ARM_SPECIMEN);
                                            outtake._setPivotPosition(OuttakeSubsystem.PIVOT_SPECIMEN_COLLECT);
                                        })
                                ),

                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(175),

                        new InstantCommand(() -> {
                            outtake._setPivotPosition(OuttakeSubsystem.PIVOT_SPECIMEN_DEPOSIT);
                            outtake._setArmPosition(OuttakeSubsystem.ARM_OUT);
                        }),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        new InstantCommand(() -> outtake._setSlidesPosition(20)),

                        new FollowPathCommand(follower, Observation.toSubmersiblePath(3)),

                        new InstantCommand(() -> outtake._setSlidesPosition(110)),
                        new InstantCommand(() -> outtake._setPivotPosition(OuttakeSubsystem.PIVOT_IN)),
                        new WaitCommand(200),

                        new FollowPathCommand(follower, Submersible.depositPath(3)).setHoldEnd(false).withTimeout(500),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),

                        new InstantCommand(() -> outtake._setSlidesPosition(0))
                )
        );
    }
}
