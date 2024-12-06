package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

@Autonomous(name = "Auto Test (Blue Specimen)", group = "Autonomous")
public class AutoTest extends CommandOpMode {
    @Override
    public void initialize() {
        Pose startingPose = new Pose(9.8, 56.5, Math.toRadians(180));

        Pose submersibleDeposit1Pose = new Pose(37.5, 67, Math.toRadians(180));

        Pose parkPose = new Pose(15.7, 20);

        Pose observationDepositPose = new Pose(14.5, 24.2, Math.toRadians(180));

        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPose);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        PathChain startToSubmersiblePath = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startingPose),
                                new Point(28.909, 56.813, Point.CARTESIAN),
                                new Point(26.391, 66.256, Point.CARTESIAN),
                                new Point(submersibleDeposit1Pose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        PathChain deposit1Path = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Point(submersibleDeposit1Pose),
                                new Point(submersibleDeposit1Pose.getX() - 5.5, submersibleDeposit1Pose.getY(), Point.CARTESIAN)
                        )
                )
                .setPathEndTimeoutConstraint(250)
                .setZeroPowerAccelerationMultiplier(10)
                .build();

        PathChain observationDeposit1Path = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(submersibleDeposit1Pose.getX() - 5.5, submersibleDeposit1Pose.getY(), Point.CARTESIAN),
                                new Point(2.300, 36.000, Point.CARTESIAN),
                                new Point(105.100, 38.200, Point.CARTESIAN),
                                new Point(65.300, 11.400, Point.CARTESIAN),
                                new Point(36.100, 32.700, Point.CARTESIAN),
                                new Point(observationDepositPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        register(intake, outtake);

        schedule(
                new RunCommand(follower::update),
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.OUT)),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT)),
                        new WaitCommand(150),
                        new FollowPathCommand(follower, startToSubmersiblePath).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT))
                                )
                        ),
                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN)),
                        new WaitCommand(850),
                        new FollowPathCommand(follower, deposit1Path).setHoldEnd(false),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),
                        new FollowPathCommand(follower, observationDeposit1Path).setHoldEnd(false).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED)),
                                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
                                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.IN))
                                )
                        )
                )
        );
    }
}
