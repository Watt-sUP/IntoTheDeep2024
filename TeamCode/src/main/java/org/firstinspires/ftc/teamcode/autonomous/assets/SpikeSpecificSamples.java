package org.firstinspires.ftc.teamcode.autonomous.assets;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public enum SpikeSpecificSamples {
    LEFT(58.5, 28),
    MIDDLE(58.5, 18),
    RIGHT(61, 12.5);

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
                                new Point(12.905, 56.184, Point.CARTESIAN),
                                new Point(31.475, 10.702, Point.CARTESIAN),
                                new Point(50.727, 60.590, Point.CARTESIAN),
                                new Point(57.757, 31.003, Point.CARTESIAN),
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
                                new Point(52.407, 30.216, Point.CARTESIAN),
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