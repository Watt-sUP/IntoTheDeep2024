package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
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
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

@Autonomous(name = "Auto Test (Blue Specimen)", group = "Autonomous")
public class AutoTest extends CommandOpMode {
    @Override
    public void initialize() {
        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(12.150, 49.307));

        PathBuilder builder = new PathBuilder();

        builder.addPath(
                        // Line 1
                        new BezierCurve(
                                new Point(12.150, 49.307, Point.CARTESIAN),
                                new Point(23.760, 49.156, Point.CARTESIAN),
                                new Point(22.743, 65.441, Point.CARTESIAN),
                                new Point(37.331, 65.441, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(37.331, 65.441, Point.CARTESIAN),
                                new Point(7.815, 22.015, Point.CARTESIAN),
                                new Point(64.510, 44.029, Point.CARTESIAN),
                                new Point(70.240, 22.919, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(70.240, 22.919, Point.CARTESIAN),
                                new Point(16.410, 23.221, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(16.410, 23.221, Point.CARTESIAN),
                                new Point(69.185, 33.324, Point.CARTESIAN),
                                new Point(70.240, 12.515, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(70.240, 12.515, Point.CARTESIAN),
                                new Point(16.862, 12.666, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(16.862, 12.666, Point.CARTESIAN),
                                new Point(39.932, 11.610, Point.CARTESIAN),
                                new Point(51.995, 24.276, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(51.995, 24.276, Point.CARTESIAN),
                                new Point(12.150, 24.276, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(12.150, 24.276, Point.CARTESIAN),
                                new Point(38.688, 50.513, Point.CARTESIAN),
                                new Point(20.292, 68.758, Point.CARTESIAN),
                                new Point(37.934, 67.099, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0));

        PathChain path = builder.build();

        schedule(
                new RunCommand(follower::update),
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new FollowPathCommand(follower, path)
                )
        );
    }
}
