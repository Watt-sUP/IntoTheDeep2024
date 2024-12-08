package org.firstinspires.ftc.teamcode.autonomous.assets;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class Observation {
    public static final double SAMPLE_TO_OBSERVATION_OFFSET = 47;

    public static final Pose prepareCollectPose = new Pose(28, 28, Math.toRadians(180));
    public static final Pose collectPose = new Pose(7, 28, Math.toRadians(180));

    public static final Pose parkPose = new Pose(12.7, 17.6);

    public static PathChain collectPath = new PathBuilder()
            .addPath(
                    new BezierLine(
                            new Point(prepareCollectPose),
                            new Point(collectPose)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setZeroPowerAccelerationMultiplier(4)
            .setPathEndTimeoutConstraint(18)
            .build();

    public static PathChain toSubmersiblePath(int pos) {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(collectPose),
                                new Point(31.370, 30.689, Point.CARTESIAN),
                                new Point(18.622, 65.941, Point.CARTESIAN),
                                new Point(Submersible.depositPose(pos, false))
                        )
                )
                .setPathEndTimeoutConstraint(3)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
}
