package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.pedropathing.localization.Pose;

public class Basket {
    public static double heading = -43;

    public static Pose getBasketPose() {
        return new Pose(15.6, 126.1, Math.toRadians(heading));
    }

    public static Pose getPreloadBasketPose() {
        return new Pose(7, 120, -90);
    }
}