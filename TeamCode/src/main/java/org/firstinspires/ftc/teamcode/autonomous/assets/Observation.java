package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class Observation {
    public static final double SAMPLE_TO_OBSERVATION_OFFSET = 44.5;

    public static final Pose prepareCollectPose = new Pose(22, 29.2, Math.toRadians(180));
    public static final Pose collectPose = new Pose(6.6, 29.2, Math.toRadians(180));

    public static final Pose parkPose = new Pose(12.7, 17.6);

    public static PathChain collectPath = new PathBuilder()
            .addPath(
                    new BezierLine(
                            new Point(prepareCollectPose),
                            new Point(collectPose)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setZeroPowerAccelerationMultiplier(12)
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
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
}
