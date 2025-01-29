package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.pedropathing.localization.Pose;

public enum SpikeSpecificSamples {
    LEFT(60, 25.5),
    MIDDLE(58.5, 15.8),
    RIGHT(58.5, 8);

    public final Pose POSE;
    private final double x, y;

    SpikeSpecificSamples(double x, double y) {
        this.x = x;
        this.y = y;
        this.POSE = new Pose(x, y, Math.toRadians(0));
    }
}