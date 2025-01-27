package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.pedropathing.localization.Pose;

public class Basket {
    public static final double DEPOSIT_OFFSET = 0.215;
    public static double HEADING = 313;

    public static Pose getBasketPose(int pos) {
        return new Pose(17 + DEPOSIT_OFFSET * pos, 124 - DEPOSIT_OFFSET * pos, Math.toRadians(HEADING));
    }
}