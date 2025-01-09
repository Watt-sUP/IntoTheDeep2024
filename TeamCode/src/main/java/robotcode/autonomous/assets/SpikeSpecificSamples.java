package robotcode.autonomous.assets;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public enum SpikeSpecificSamples {
    LEFT(52.9, 27),
    MIDDLE(52.9, 17),
    RIGHT(52.9, 11.8);

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
                                new Point(6.445, 64.769, Point.CARTESIAN),
                                new Point(15.563, 2.201, Point.CARTESIAN),
                                new Point(72.314, 55.179, Point.CARTESIAN),
                                new Point(55.651, 28.611, Point.CARTESIAN),
                                new Point(this.x, this.y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public PathChain getObservationToSamplePath(SpikeSpecificSamples startPosition) {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(startPosition.x - Observation.SAMPLE_TO_OBSERVATION_OFFSET, startPosition.y, Point.CARTESIAN),
                                new Point(56.693, 24.621, Point.CARTESIAN),
                                new Point(58.961, 24.459, Point.CARTESIAN),
                                new Point(this.x, this.y, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                .setZeroPowerAccelerationMultiplier(2)
                .build();
    }

    public PathChain goIntoPrepareCollectPath() {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(this.x - Observation.SAMPLE_TO_OBSERVATION_OFFSET, this.y, Point.CARTESIAN),
                                new Point(29.000, 18.000, Point.CARTESIAN),
                                new Point(Observation.prepareCollectPose)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
}