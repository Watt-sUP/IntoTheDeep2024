package robotcode.autonomous.assets;

import robotcode.pedroPathing.localization.Pose;

public class AutonomousConstants {
    public static final double START_X = 8.8;
    public static final Pose START_POSE_SPECIMEN = new Pose(START_X, 56.5, Math.toRadians(180));
    public static final Pose START_POSE_BASKET = new Pose(START_X, 80, Math.toRadians(180));
}
