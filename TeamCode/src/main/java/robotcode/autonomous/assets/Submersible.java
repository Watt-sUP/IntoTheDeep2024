package robotcode.autonomous.assets;

import static robotcode.autonomous.assets.AutonomousConstants.START_POSE_BASKET;
import static robotcode.autonomous.assets.AutonomousConstants.START_POSE_SPECIMEN;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class Submersible {
    public static final double SUBMERSIBLE_DEPOSIT_X_OFFSET = 3;
    public static final double SUBMERSIBLE_DEPOSIT_Y_OFFSET = 3;
    public static final double SUBMERSIBLE_DEPOSIT_BACKWARD = 6.2;
    public static final Pose POSE = new Pose(37.2, 67.6);
    public static PathChain startPathSpecimen = new PathBuilder()
            .addPath(
                    new BezierCurve(
                            new Point(START_POSE_SPECIMEN),
                            new Point(24.078, 67.514, Point.CARTESIAN),
                            new Point(Submersible.depositPose(0, false))
                    )
            )
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
                .setPathEndTimeoutConstraint(15)
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
                .build();
    }
}
