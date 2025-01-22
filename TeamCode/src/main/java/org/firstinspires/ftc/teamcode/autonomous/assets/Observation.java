package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class Observation {
    public static final double SAMPLE_TO_OBSERVATION_OFFSET = 43;
    public static final double COLLECT_OFFSET = 0.2;
    public static final Pose parkPose = new Pose(16, 32, Math.toRadians(210));

    ;
    public static PathChain samplesToObservationPath = new PathBuilder()
            .addPath(
                    // Submersible to Left
                    new BezierCurve(
                            new Point(Submersible.depositPose(0, false)),
                            new Point(22.316, 26.689, Point.CARTESIAN),
                            new Point(48.704, 48.251, Point.CARTESIAN),
                            new Point(SpikeSpecificSamples.LEFT.POSE)
                    )
            )
            .setPathEndTValueConstraint(2.15 / 3.0)
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .addPath(
                    // Left to Observation
                    new BezierLine(
                            new Point(SpikeSpecificSamples.LEFT.POSE),
                            new Point(18.000, SpikeSpecificSamples.LEFT.POSE.getY(), Point.CARTESIAN)
                    )
            )
            .setPathEndTValueConstraint(0.8)
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .addPath(
                    // Left to Middle
                    new BezierCurve(
                            new Point(18.000, SpikeSpecificSamples.LEFT.POSE.getY(), Point.CARTESIAN),
                            new Point(49.000, 32.500, Point.CARTESIAN),
                            new Point(SpikeSpecificSamples.MIDDLE.POSE)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setPathEndTValueConstraint(2.0 / 3.0)
            .addPath(
                    // Middle to Observation
                    new BezierLine(
                            new Point(SpikeSpecificSamples.MIDDLE.POSE),
                            new Point(18.000, SpikeSpecificSamples.MIDDLE.POSE.getY(), Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setPathEndTValueConstraint(0.8)
            .addPath(
                    // Middle to Right
                    new BezierCurve(
                            new Point(18.000, SpikeSpecificSamples.MIDDLE.POSE.getY(), Point.CARTESIAN),
                            new Point(49.000, 21.800, Point.CARTESIAN),
                            new Point(SpikeSpecificSamples.RIGHT.POSE)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setPathEndTValueConstraint(2.0 / 3.0)
            .addPath(
                    // Right to Observation
                    new BezierLine(
                            new Point(SpikeSpecificSamples.RIGHT.POSE),
                            new Point(18.000, SpikeSpecificSamples.RIGHT.POSE.getY(), Point.CARTESIAN)
                    )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .setPathEndTValueConstraint(0.5)
            .build();

    public static Pose prepareCollectPose(int pos) {
        return new Pose(22.5, 27 + pos * COLLECT_OFFSET, Math.toRadians(180));
    }

    public static Pose collectPose(int pos) {
        return new Pose(3.2, 27 + pos * COLLECT_OFFSET, Math.toRadians(180));
    }
}
