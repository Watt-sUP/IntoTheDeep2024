package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

public class FollowPointCommand extends CommandBase {
    private final Follower follower;
    private final Pose target;
    private double magnitude;

    public FollowPointCommand(Follower follower, Pose target, double magnitude) {
        this.follower = follower;
        this.target = target;
        this.magnitude = magnitude;
    }

    public FollowPointCommand(Follower follower, Pose target) {
        this(follower, target, 7);
    }

    @Override
    public void initialize() {
        follower.holdPoint(target);
    }

    @Override
    public boolean isFinished() {
        return follower.getTranslationalError().getMagnitude() < magnitude && follower.headingError < Math.toRadians(2.5);
    }
}
