package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.pedropathing.localization.Pose;

public enum SpikeYellowSamples {
    LEFT(24, 119.5, 0),
    MIDDLE(24, 129.4, 0),
    RIGHT(44.2, 117.6, 90);

    public final double x, y, heading;

    SpikeYellowSamples(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose getPose() {
        return new Pose(this.x, this.y, Math.toRadians(this.heading));
    }
}
