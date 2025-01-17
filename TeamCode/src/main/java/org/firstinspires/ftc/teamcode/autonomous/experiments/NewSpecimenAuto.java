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

import org.firstinspires.ftc.teamcode.autonomous.assets.Submersible;
import org.firstinspires.ftc.teamcode.commands.FollowPointCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

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

                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPointCommand(follower, new Pose(25.900, 62.250, Math.toRadians(180))),
                        new FollowPointCommand(follower, Submersible.POSE, 0.005),

                        new WaitCommand(250),

                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),

                        new WaitCommand(100),

                        new FollowPointCommand(follower, new Pose(31.2, 34.7, Math.toRadians(311)), 0.1).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
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
                        new WaitCommand(850),
                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.RIGHT)),
                        new WaitCommand(150),
                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(100),

                        new FollowPointCommand(follower, new Pose(24, 34.7, Math.toRadians(240)), 0.1)
                )
        );
    }
}
