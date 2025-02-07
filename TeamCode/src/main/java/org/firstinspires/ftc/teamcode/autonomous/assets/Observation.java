package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class Observation {
    public static final double OBSERVATION_X = 19;
    public static final double COLLECT_OFFSET = 0;
    public static final Pose parkPose = new Pose(20, 38, Math.toRadians(240));

    public static PathChain samplesToObservationPath = new PathBuilder()
            .addPath(
                    // Submersible to Left
                    new BezierCurve(
                            new Point(Submersible.depositPose(0, false)),
                            new Point(16, 26, Point.CARTESIAN),
                            new Point(49, 49, Point.CARTESIAN),
                            new Point(SpikeSpecificSamples.LEFT.POSE)
                    )
            )
            .setPathEndTValueConstraint(2.5 / 3.0)
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .addPath(
                    // Left to Observation
                    new BezierLine(
                            new Point(SpikeSpecificSamples.LEFT.POSE),
                            new Point(OBSERVATION_X, SpikeSpecificSamples.LEFT.POSE.getY(), Point.CARTESIAN)
                    )
            )
            .setPathEndTValueConstraint(0.82)
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .addPath(
                    // Left to Middle
                    new BezierCurve(
                            new Point(OBSERVATION_X, SpikeSpecificSamples.LEFT.POSE.getY(), Point.CARTESIAN),
                            new Point(49.000, 32.500, Point.CARTESIAN),
                            new Point(SpikeSpecificSamples.MIDDLE.POSE)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setPathEndTValueConstraint(2.5 / 3.0)
            .addPath(
                    // Middle to Observation
                    new BezierLine(
                            new Point(SpikeSpecificSamples.MIDDLE.POSE),
                            new Point(OBSERVATION_X, SpikeSpecificSamples.MIDDLE.POSE.getY(), Point.CARTESIAN)
                    )
            )
            .setPathEndTValueConstraint(0.82)
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .addPath(
                    // Middle to Right
                    new BezierCurve(
                            new Point(OBSERVATION_X, SpikeSpecificSamples.MIDDLE.POSE.getY(), Point.CARTESIAN),
                            new Point(49.000, 21.800, Point.CARTESIAN),
                            new Point(SpikeSpecificSamples.RIGHT.POSE)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setPathEndTValueConstraint(2.5 / 3.0)
            .addPath(
                    // Right to Observation
                    new BezierLine(
                            new Point(SpikeSpecificSamples.RIGHT.POSE),
                            new Point(OBSERVATION_X, SpikeSpecificSamples.RIGHT.POSE.getY(), Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setPathEndTValueConstraint(0.9)
            .build();

    public static Pose prepareCollectPose(int pos) {
        return new Pose(22.5, 26.5 + pos * COLLECT_OFFSET, Math.toRadians(180));
    }

    public static Pose collectPose(int pos) {
        return new Pose(9.55, 26.5 + pos * COLLECT_OFFSET, Math.toRadians(180));
    }
}
