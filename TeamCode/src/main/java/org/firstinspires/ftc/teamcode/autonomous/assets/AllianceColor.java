package org.firstinspires.ftc.teamcode.autonomous.assets;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public enum AllianceColor {
    RED, BLUE;

    private final Pose startingPose = new Pose(8.6, 56.5, Math.toRadians(180));

    public Pose convertPose(Pose pose) {
        if (this == BLUE)
            return pose;
        else
            return new Pose(144 - pose.getX(), 144 - pose.getY(), Math.PI - pose.getHeading());
    }

    public Point convertPoint(Point point) {
        return this == BLUE ? point : new Point(144 - point.getX(), 144 - point.getY(), Point.CARTESIAN);
    }

    public Pose getStartingPose() {
        return this.convertPose(startingPose);
    }
}