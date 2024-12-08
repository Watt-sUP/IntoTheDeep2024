package org.firstinspires.ftc.teamcode.autonomous.assets;

import static org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousConstants.START_POSE_BASKET;
import static org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousConstants.START_POSE_SPECIMEN;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class Submersible {
    public static final double SUBMERSIBLE_DEPOSIT_X_OFFSET = 3;
    public static final double SUBMERSIBLE_DEPOSIT_Y_OFFSET = 3;
    public static final double SUBMERSIBLE_DEPOSIT_BACKWARD = 6.5;
    public static final Pose POSE = new Pose(37.5, 67);

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

    public static PathChain startPathSpecimen = new PathBuilder()
            .addPath(
                    new BezierCurve(
                            new Point(START_POSE_SPECIMEN),
                            new Point(21.927, 65.784, Point.CARTESIAN),
                            new Point(Submersible.depositPose(0, false))
                    )
            )
            .setPathEndTimeoutConstraint(3)
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
                .setZeroPowerAccelerationMultiplier(10)
                .build();
    }
}
