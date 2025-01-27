package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.pedropathing.localization.Pose;

public enum SpikeYellowSamples {
    LEFT(24, 120, 0),
    MIDDLE(24, 129.4, 0),
    RIGHT(44.2, 117.8, 90);

    public final double x, y, heading;
    public final Pose POSE;

    SpikeYellowSamples(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.POSE = new Pose(x, y, Math.toRadians(heading));
    }
}
