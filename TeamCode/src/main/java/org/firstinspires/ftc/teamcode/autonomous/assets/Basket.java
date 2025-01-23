package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.pedropathing.localization.Pose;

public class Basket {
    public static final double DEPOSIT_OFFSET = 0.3;
    public static double heading = -43;

    public static Pose getBasketPose(int pos) {
        return new Pose(17 + DEPOSIT_OFFSET * pos, 124 - DEPOSIT_OFFSET * pos, Math.toRadians(heading));
    }
}