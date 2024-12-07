package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

@Autonomous(name = "Auto Test (Blue Specimen)", group = "Autonomous")
public class AutoTest extends CommandOpMode {
    private static final double SUBMERSIBLE_DEPOSIT_Y_OFFSET = 3;
    private static final double SUBMERSIBLE_DEPOSIT_BACKWARD = 6.5;

    private static final double SAMPLE_TO_OBSERVATION_OFFSET = 48;

    private static final Pose prepareCollectPose = new Pose(28, 28, Math.toRadians(180));
    private static final Pose collectPose = new Pose(7, 28, Math.toRadians(180));

    private Pose sumbersibleDepositPose(int pos, boolean deposit) {
        return new Pose(37.5 - (deposit ? SUBMERSIBLE_DEPOSIT_BACKWARD : 0) - pos * 3, 67 + pos * SUBMERSIBLE_DEPOSIT_Y_OFFSET, Math.toRadians(180));
    }

    private PathChain observationToSubmersible(int pos) {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(collectPose),
                                new Point(31.370, 30.689, Point.CARTESIAN),
                                new Point(18.622, 65.941, Point.CARTESIAN),
                                new Point(sumbersibleDepositPose(pos, false))
                        )
                )
                .setPathEndTimeoutConstraint(3)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private PathChain submersibleDeposit(int pos) {
        return new PathBuilder().addPath(
                        new BezierLine(
                                new Point(sumbersibleDepositPose(pos, false)),
                                new Point(sumbersibleDepositPose(pos, true))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(10)
                .setZeroPowerAccelerationMultiplier(10)
                .build();
    }


    @Override
    public void initialize() {
        Pose startingPose = new Pose(8.8, 56.5, Math.toRadians(180));

        Pose parkPose = new Pose(12.7, 17.6);

        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPose);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        PathChain startToSubmersiblePath = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startingPose),
                                new Point(28.9, 56.8, Point.CARTESIAN),
                                new Point(26.3, 66.2, Point.CARTESIAN),
                                new Point(sumbersibleDepositPose(0, false))
                        )
                )
                .setPathEndTimeoutConstraint(3)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain collectFromObservationPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(prepareCollectPose),
                                new Point(collectPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(4)
                .setPathEndTimeoutConstraint(18)
                .build();

        register(intake, outtake);

        schedule(
                new RunCommand(follower::update),
                new RunCommand(() -> {
                    if (!follower.isBusy()) {
                        return;
                    }

                    follower.telemetryDebug(telemetry);
                }),
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
                        new FollowPathCommand(follower, startToSubmersiblePath).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(350),
                                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN)),
                                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.OUT))
                                )
                        ),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT)),
                        new WaitCommand(450),
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPathCommand(follower, submersibleDeposit(0)).setHoldEnd(false),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),
                        new InstantCommand(() -> follower.setMaxPower(0.6)),
                        new FollowPathCommand(follower,
                                SpikeSamplePositions.LEFT.getSubmersibleToSamplePath(
                                        sumbersibleDepositPose(0, true)
                                )
                        ).setHoldEnd(true).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED)),
                                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
                                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.IN))
                                )
                        ),
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPathCommand(follower, SpikeSamplePositions.LEFT.getPathToObservation()).setHoldEnd(false),
                        new InstantCommand(() -> follower.setMaxPower(0.6)),
                        new FollowPathCommand(follower, SpikeSamplePositions.MIDDLE.getObservationToSamplePath(SpikeSamplePositions.LEFT)).setHoldEnd(false),
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPathCommand(follower, SpikeSamplePositions.MIDDLE.getPathToObservation()).setHoldEnd(false),
                        new FollowPathCommand(follower, SpikeSamplePositions.MIDDLE.goIntoPrepareCollectPath()).alongWith(
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
                        new ParallelRaceGroup(
                                new FollowPathCommand(follower, collectFromObservationPath).setHoldEnd(false),
                                new WaitCommand(2500)
                        ),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(250),
                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.OUT)),
                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN)),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.OUT)),
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPathCommand(follower, observationToSubmersible(1)),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT)),
                        new WaitCommand(450),
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPathCommand(follower, submersibleDeposit(1)).setHoldEnd(false),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),
                        new FollowPathCommand(follower, follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Point(sumbersibleDepositPose(1, true)),
                                                new Point(31.684, 45.325, Point.CARTESIAN),
                                                new Point(parkPose)
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .setPathEndTimeoutConstraint(15)
                                .build()
                        ).alongWith(new SequentialCommandGroup(
                                new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED)),
                                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
                                new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.IN))
                        ))
                )
        );
    }

    private enum SpikeSamplePositions {
        LEFT(61, 28),
        MIDDLE(61, 18),
        RIGHT(61, 12.5);

        private final double x, y;

        SpikeSamplePositions(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public PathChain getSubmersibleToSamplePath(Pose startPose) {
            return new PathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Point(startPose),
                                    new Point(23.816, 53.666, Point.CARTESIAN),
                                    new Point(32.314, 6.295, Point.CARTESIAN),
                                    new Point(50.727, 60.590, Point.CARTESIAN),
                                    new Point(60.484, 30.059, Point.CARTESIAN),
                                    new Point(this.x, this.y, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }

        public PathChain getObservationToSamplePath(SpikeSamplePositions startPosition) {
            return new PathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Point(startPosition.x - SAMPLE_TO_OBSERVATION_OFFSET, startPosition.y, Point.CARTESIAN),
                                    new Point(64.419, 33.049, Point.CARTESIAN),
                                    new Point(this.x, this.y, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }

        public PathChain getPathToObservation() {
            return new PathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Point(this.x, this.y, Point.CARTESIAN),
                                    new Point(this.x - SAMPLE_TO_OBSERVATION_OFFSET, this.y, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .setZeroPowerAccelerationMultiplier(2)
                    .build();
        }

        public PathChain goIntoPrepareCollectPath() {
            return new PathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Point(this.x - SAMPLE_TO_OBSERVATION_OFFSET, this.y, Point.CARTESIAN),
                                    new Point(29.000, 18.000, Point.CARTESIAN),
                                    new Point(prepareCollectPose)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
}
