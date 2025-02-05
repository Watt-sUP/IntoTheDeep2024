package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.pedropathing.localization.Pose;

public enum SpikeYellowSamples {
    LEFT(18, 117, 0),
    MIDDLE(18, 126.5, 0),
    RIGHT(44, 108.5, 90);

    public final double x, y, heading;
    public final Pose POSE;

    SpikeYellowSamples(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.POSE = new Pose(x, y, Math.toRadians(heading));
    }
}
