package robotcode.autonomous.assets;

import robotcode.pedroPathing.localization.Pose;
import robotcode.pedroPathing.pathGeneration.BezierCurve;
import robotcode.pedroPathing.pathGeneration.BezierLine;
import robotcode.pedroPathing.pathGeneration.PathBuilder;
import robotcode.pedroPathing.pathGeneration.PathChain;
import robotcode.pedroPathing.pathGeneration.Point;

public enum SpikeSpecificSamples {
    LEFT(55.6, 26.4),
    MIDDLE(55.6, 15),
    RIGHT(55.6, 8);

    private final double x, y;

    SpikeSpecificSamples(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public PathChain getSubmersibleToSamplePath(Pose startPose) {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startPose),
                                new Point(13.362, 16.978, Point.CARTESIAN),
                                new Point(62.725, 55.179, Point.CARTESIAN),
                                new Point(this.x, this.y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTValueConstraint(0.92)
                .build();
    }

    public PathChain getObservationToSamplePath(SpikeSpecificSamples startPosition) {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startPosition.x - Observation.SAMPLE_TO_OBSERVATION_OFFSET, startPosition.y, Point.CARTESIAN),
                                new Point(60.052, 36.943, Point.CARTESIAN),
                                new Point(this.x, this.y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTValueConstraint(0.9)
                .build();
    }

    public PathChain getPathToObservation() {
        return new PathBuilder()
                .addPath(
                        new BezierLine(
                                new Point(this.x, this.y, Point.CARTESIAN),
                                new Point(this.x - Observation.SAMPLE_TO_OBSERVATION_OFFSET, this.y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public PathChain goIntoPrepareCollectPath() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(this.x - Observation.SAMPLE_TO_OBSERVATION_OFFSET, this.y, Point.CARTESIAN),
                                new Point(25.000, 18.000, Point.CARTESIAN),
                                new Point(Observation.prepareCollectPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
}