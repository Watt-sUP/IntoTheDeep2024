package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Drawing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

import java.util.Locale;

/**
 * This command handles updates to the Pedro follower from Limelight detections.
 * It is assumed that the bot's initial position has been set before this command runs.
 */
public class LimelightRelocalization extends CommandBase {
    private final LimelightSubsystem limelight;
    private final Telemetry dashboardTelemetry;
    private final Follower follower;
    private boolean disableUpdates = false;

    public LimelightRelocalization(LimelightSubsystem limelight, Follower follower) {
        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
        this.limelight = limelight;
        this.follower = follower;
        addRequirements(limelight);
    }

    public LimelightRelocalization enableUpdates(boolean set) {
        disableUpdates = !set;
        return this;
    }

    public void initialize() {
        final int PIPELINE_ID = 0;
        limelight.setPipeline(PIPELINE_ID);
    }

    public void execute() {
        // Subtract 90 degrees due to pedro conversion
        double botHeading = Math.toDegrees(follower.getPose().getHeading() - Math.PI / 2);
        Pose botPose = limelight.getBotPose(botHeading);

        if (botPose != null) {
            if (!disableUpdates)
                follower.setCurrentPoseWithOffset(LimelightSubsystem.toPedroPoseNeutral(botPose));
            else {
                Drawing.drawRobot(botPose, "#7a120b");
                Drawing.drawRobot(LimelightSubsystem.fromPedroPose(follower.getPose()), "#4CAF50");
                Drawing.sendPacket();
            }

            dashboardTelemetry.addData("Limelight Position",
                    String.format(Locale.ROOT, "(X=%.3f, Y=%.3f)", botPose.getX(), botPose.getY()));
            dashboardTelemetry.update();
        } else {
            dashboardTelemetry.addLine("No AprilTag in sight!");
            dashboardTelemetry.update();
        }
    }
}
