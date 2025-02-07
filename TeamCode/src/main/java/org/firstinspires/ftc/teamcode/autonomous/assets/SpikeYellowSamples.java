package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.pedropathing.localization.Pose;

public enum SpikeYellowSamples {
    LEFT(18.2, 117.25, 0),
    MIDDLE(18.2, 126.8, 0),
    RIGHT(44, 109.55, 90);

    public final double x, y, heading;
    public final Pose POSE;

    SpikeYellowSamples(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.POSE = new Pose(x, y, Math.toRadians(heading));
    }
}
