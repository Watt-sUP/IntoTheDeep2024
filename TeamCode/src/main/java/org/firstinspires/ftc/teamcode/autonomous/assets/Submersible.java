package org.firstinspires.ftc.teamcode.autonomous.assets;

import static org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousConstants.START_POSE_BASKET;
import static org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousConstants.START_POSE_SPECIMEN;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class Submersible {
    public static final double SUBMERSIBLE_DEPOSIT_X_OFFSET = 0.65;
    public static final double SUBMERSIBLE_DEPOSIT_Y_OFFSET = 3;
    public static final double SUBMERSIBLE_DEPOSIT_BACKWARD = 5.2;
    public static final Pose POSE = new Pose(41.2, 62.25, Math.toRadians(180));
    public static PathChain startPathSpecimen = new PathBuilder()
            .addPath(
                    new BezierCurve(
                            new Point(START_POSE_SPECIMEN),
                            new Point(19, 63.5, Point.CARTESIAN),
                            new Point(POSE)
                    )
            )
            .setPathEndTValueConstraint(0.85)
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();
    public static PathChain startPathBasket = new PathBuilder()
            .addPath(
                    new BezierCurve(
                            new Point(START_POSE_BASKET),
                            new Point(19.200, 76.013, Point.CARTESIAN),
                            new Point(Submersible.depositPose(3, false))
                    )
            )
            .setPathEndTimeoutConstraint(3)
            .setConstantHeadingInterpolation(Math.toRadians(180))
            .build();

    public static PathChain toObservationParkPath(int pos) {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(depositPose(pos, true)),
                                new Point(20.511, 45.639, Point.CARTESIAN),
                                new Point(Observation.parkPose)
                        )
                )
                .setTangentHeadingInterpolation()
                .setPathEndTValueConstraint(0.86)
                .build();
    }

    public static PathChain toCollectPath(int pos) {
        return new PathBuilder()
                .addPath(
                        new BezierCurve(
                                new Point(depositPose(pos, true)),
                                new Point(2.515, 50.148, Point.CARTESIAN),
                                new Point(57.380, 22.795, Point.CARTESIAN),
                                new Point(29.869, 29.240, Point.CARTESIAN),
                                new Point(new Pose(Observation.collectPose.getX() - 0.6 * pos, Observation.collectPose.getY() - 2))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    public static Pose depositPose(int pos, boolean deposit) {
        return new Pose(
                POSE.getX() - (deposit ? SUBMERSIBLE_DEPOSIT_BACKWARD : 0) - pos * SUBMERSIBLE_DEPOSIT_X_OFFSET,
                POSE.getY() + pos * SUBMERSIBLE_DEPOSIT_Y_OFFSET,
                Math.toRadians(180)
        );
    }

    public static PathChain depositPath(int pos) {
        return new PathBuilder().addPath(
                        new BezierLine(
                                new Point(depositPose(pos, false)),
                                new Point(depositPose(pos, true))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(10)
                .setPathEndTValueConstraint(0.88)
                .build();
    }
}
