package org.firstinspires.ftc.teamcode.autonomous.experiments;

import static org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousConstants.START_POSE_SPECIMEN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.assets.Observation;
import org.firstinspires.ftc.teamcode.autonomous.assets.Submersible;
import org.firstinspires.ftc.teamcode.commands.FollowPointCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

@Autonomous(name = "New Specimen Autonomous", group = "Auto Experiments")
public class NewSpecimenAuto extends CommandOpMode {
    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.setConstants(FConstants.class, LConstants.class);

        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE_SPECIMEN);
//
//        SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
//        otos.calibrateImu();
//        otos.setOffset(new SparkFunOTOS.Pose2D(-6.8897637795275, 0, 0));
//        otos.setAngularUnit(AngleUnit.RADIANS);
//        otos.setAngularScalar(.992);
//        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
//        otos.resetTracking();

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        register(intake, outtake);

        schedule(
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new RunCommand(() -> {
                            follower.update();
                            try {
                                follower.telemetryDebug(telemetry);
                            } catch (RuntimeException ignored) {
                            }
                        })
                ),
//                new RunCommand(() -> follower.setCurrentPoseWithOffset(new Pose(follower.getPose().getX(), follower.getPose().getY(), otos.getPosition().h))),
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),

                        new InstantCommand(() -> {
                            intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);
                            outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED);

                            outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN);

                            outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                            outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT);
                        }),

                        new WaitCommand(500),

                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPointCommand(follower, new Pose(25.900, 62.250, Math.toRadians(180))),
                        new FollowPointCommand(follower, Submersible.POSE, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(150),
                                                new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT)),
                                                new WaitCommand(350),
                                                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
                                                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED))
                                        )
                                ),
                        new WaitCommand(150),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),
                        new WaitCommand(100),

                        new FollowPointCommand(follower, Submersible.depositPose(0, true), 0.8),

                        new InstantCommand(() -> {
                            outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                            outtake.setArmState(OuttakeSubsystem.ArmState.IN);
                            outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                        }),

                        new FollowPointCommand(follower, Observation.parkPose, 9)

//                        new FollowPointCommand(follower, SpikeSpecificSamples.LEFT.POSE, 0.7)
//                                .alongWith(
//                                        new SequentialCommandGroup(
//                                                new WaitCommand(500),
//                                                new InstantCommand(() -> {
//                                                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
//                                                    outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER);
//                                                    outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
//                                                })
//
//                                        )
//                                ),
//
//                        new WaitCommand(250),
//                        new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT)),
//                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.DOWN)),
//                        new WaitCommand(250),
//                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
//                        new WaitCommand(200),
//                        new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
//                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
//
//                        new FollowPointCommand(follower, SpikeSpecificSamples.MIDDLE.POSE, 0.7)
//                                .alongWith(
//                                        new SequentialCommandGroup(
//                                                new WaitCommand(250),
//                                                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
//                                                new WaitCommand(250),
//                                                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
//                                                new WaitCommand(100),
//                                                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),
//                                                new WaitCommand(150),
//                                                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.OUT)),
//                                                new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT)),
//                                                new WaitCommand(150),
//                                                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT)),
//                                                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.COLLECT)),
//                                                new WaitCommand(500),
//                                                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),
//                                                new WaitCommand(200),
//                                                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER)),
//                                                new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.IN))
//                                        )
//                                ),
//
//                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.DOWN)),
//                        new WaitCommand(100),
//                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
//                        new WaitCommand(200),
//                        new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
//                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
//                        new WaitCommand(250),
//                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
//                        new WaitCommand(250),
//                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
//                        new WaitCommand(100),
//                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),
//                        new WaitCommand(150),
//                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.OUT)),
//                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT)),
//                        new WaitCommand(150),
//                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.DOWN)),
//                        new WaitCommand(500),
//                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),
//                        new WaitCommand(200),
//                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN)),
//                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT)),
//
//                        new FollowPathCommand(follower, follower.pathBuilder()
//                                .addPath(
//                                        new BezierCurve(
//                                                new Point(SpikeSpecificSamples.MIDDLE.POSE),
//                                                new Point(62.500, 14.500, Point.CARTESIAN),
//                                                new Point(SpikeSpecificSamples.RIGHT.POSE)
//                                        )
//                                )
//                                .setConstantHeadingInterpolation(Math.toRadians(0))
//                                .setPathEndTValueConstraint(0.75)
//                                .build()
//                        ).setHoldEnd(false),
//
//                        new FollowPointCommand(follower, new Pose(SpikeSpecificSamples.RIGHT.POSE.getX() - Observation.SAMPLE_TO_OBSERVATION_OFFSET, SpikeSpecificSamples.RIGHT.POSE.getY(), Math.toRadians(0)), 2),
//
//                        new FollowPointCommand(follower, Observation.prepareCollectPose, 1),
//                        new FollowPointCommand(follower, Observation.collectPose, 1)
                )
        );
    }
}
