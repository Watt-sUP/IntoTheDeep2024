package robotcode.autonomous.experiments;

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
import com.pedropathing.pathgen.BezierCurve;
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
                            try {
                                follower.telemetryDebug(telemetry);
                            } catch (RuntimeException ignored) {
                            }
                        })
                ),
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),

                        new InstantCommand(() -> {
                            intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);
                            outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED);

                            outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN);

                            outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                            outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT);
                        }),

                        new WaitCommand(350),

                        new InstantCommand(() -> follower.setMaxPower(0.825)),
                        new FollowPathCommand(follower, Submersible.startPathSpecimen),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),

                        new InstantCommand(() -> follower.setMaxPower(1)),

                        new FollowPathCommand(follower, follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Point(Submersible.POSE),
                                                new Point(27.000, 53.000, Point.CARTESIAN),
                                                new Point(31.2, 34, Point.CARTESIAN)
                                        )
                                )
                                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(311))
                                .setPathEndTValueConstraint(0.6)
                                .build()
                        ).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(350),
                                        new InstantCommand(() -> {
                                            outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);

                                            intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT);
                                            intake.setPivotState(IntakeSubsystem.PivotState.COLLECT);
                                            intake.setClawState(IntakeSubsystem.ClawState.OPENED);
                                        }),
                                        new WaitCommand(350),
                                        new InstantCommand(() -> {
                                            outtake.setArmState(OuttakeSubsystem.ArmState.SPECIMEN);
                                            outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT);
                                        })
                                )
                        ),
                        new InstantCommand(() -> {
                            intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT);
                            intake.setPivotState(IntakeSubsystem.PivotState.DOWN);

                        }),
                        new WaitCommand(450),
                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.RIGHT)),
                        new WaitCommand(150),
                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED))
                )
        );
    }
}
