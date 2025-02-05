package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.pedropathing.localization.Pose;

public class Basket {
    public static final double DEPOSIT_OFFSET = 0.1;
    public static double HEADING = 315;

    public static Pose getPose(int pos) {
        return new Pose(14 + DEPOSIT_OFFSET * pos, 124 - DEPOSIT_OFFSET * pos, Math.toRadians(HEADING));
    }
}