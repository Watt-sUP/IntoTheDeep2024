package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.pedropathing.localization.Pose;

public enum SpikeYellowSamples {
    LEFT(22.6, 125, 0),
    MIDDLE(22.6, 135, 0),
    RIGHT(27.5, 126.2, 45);

    public final double x, y, heading;

    SpikeYellowSamples(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose getPose() {
        return new Pose(this.x, this.y, this.heading);
    }
}
