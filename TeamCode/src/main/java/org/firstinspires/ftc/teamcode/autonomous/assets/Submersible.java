package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.pedropathing.localization.Pose;

public class Submersible {
    public static final double SUBMERSIBLE_DEPOSIT_X_OFFSET = 0.25;
    public static final double SUBMERSIBLE_DEPOSIT_Y_OFFSET = 2.2;
    public static final double SUBMERSIBLE_DEPOSIT_BACKWARD = 10;
    public static final Pose POSE = new Pose(37.2, 66.2, Math.toRadians(180));

    public static final Pose ascentParkPose = new Pose(66, 92.2, Math.toRadians(90));

    public static Pose depositPose(int pos, boolean backward) {
        return new Pose(
                POSE.getX() - (backward ? SUBMERSIBLE_DEPOSIT_BACKWARD : 0) - pos * SUBMERSIBLE_DEPOSIT_X_OFFSET,
                POSE.getY() + pos * SUBMERSIBLE_DEPOSIT_Y_OFFSET,
                Math.toRadians(180)
        );
    }
}
