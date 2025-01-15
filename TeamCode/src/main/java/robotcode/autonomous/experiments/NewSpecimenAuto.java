package robotcode.autonomous.experiments;

import static robotcode.autonomous.assets.AutonomousConstants.START_POSE_SPECIMEN;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import robotcode.autonomous.assets.Submersible;
import robotcode.commands.FollowPathCommand;
import robotcode.pedroPathing.constants.FConstants;
import robotcode.pedroPathing.constants.LConstants;
import robotcode.subsystems.IntakeSubsystem;
import robotcode.subsystems.OuttakeSubsystem;
import robotcode.util.FixedSequentialCommandGroup;

@Autonomous(name = "New Specimen Autononous", group = "Auto Experiments")
public class NewSpecimenAuto extends CommandOpMode {
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

                        new InstantCommand(() -> follower.setMaxPower(0.6)),
                        new FollowPathCommand(follower, Submersible.startPathSpecimen),

                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPathCommand(follower, Submersible.depositPath(0))
                                .setHoldEnd(false)
                                .withTimeout(600),

                        new FollowPathCommand(follower,
                                follower
                                        .pathBuilder()
                                        .addPath(
                                                new BezierCurve(
                                                        new Point(Submersible.depositPose(0, true)),
                                                        new Point(22.323, 53.450, Point.CARTESIAN),
                                                        new Point(29.826, 40.208, Point.CARTESIAN)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(311))
                                        .build()
                        ),

                        new FollowPathCommand(follower,
                                follower
                                        .pathBuilder()
                                        .addPath(
                                                new BezierCurve(
                                                        new Point(29.826, 40.208, Point.CARTESIAN),
                                                        new Point(31.139, 37.729, Point.CARTESIAN),
                                                        new Point(24.323, 35.210, Point.CARTESIAN)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(Math.toRadians(311), Math.toRadians(206))
                                        .build()
                        ),

                        new FollowPathCommand(follower,
                                follower
                                        .pathBuilder()
                                        .addPath(
                                                new BezierCurve(
                                                        new Point(24.323, 35.210, Point.CARTESIAN),
                                                        new Point(22.952, 28.454, Point.CARTESIAN),
                                                        new Point(28.288, 28.734, Point.CARTESIAN)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(Math.toRadians(206), Math.toRadians(316))
                                        .build()
                        ),

                        new FollowPathCommand(follower,
                                follower
                                        .pathBuilder()
                                        .addPath(
                                                new BezierLine(
                                                        new Point(28.288, 28.734, Point.CARTESIAN),
                                                        new Point(24.539, 28.337, Point.CARTESIAN)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(Math.toRadians(316), Math.toRadians(216))
                                        .build()
                        ),

                        new FollowPathCommand(follower,
                                follower
                                        .pathBuilder()
                                        .addPath(
                                                new BezierCurve(
                                                        new Point(24.539, 28.337, Point.CARTESIAN),
                                                        new Point(26.737, 23.895, Point.CARTESIAN),
                                                        new Point(31.880, 23.003, Point.CARTESIAN)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(305))
                                        .build()
                        ),

                        new FollowPathCommand(follower,
                                follower
                                        .pathBuilder()
                                        .addPath(
                                                new BezierCurve(
                                                        new Point(31.880, 23.003, Point.CARTESIAN),
                                                        new Point(25.165, 20.437, Point.CARTESIAN),
                                                        new Point(22.000, 29.200, Point.CARTESIAN)
                                                )
                                        )
                                        .setLinearHeadingInterpolation(Math.toRadians(305), Math.toRadians(180))
                                        .build()
                        )
                )
        );
    }
}
