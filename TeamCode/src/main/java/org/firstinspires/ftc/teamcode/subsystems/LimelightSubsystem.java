package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import javax.annotation.Nullable;

public class LimelightSubsystem extends SubsystemBase {

    private final Limelight3A limelight;

    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    public static Pose toPedroPose(Pose pose) {
        return new Pose(Math.abs(-pose.getY() - 72), pose.getX() + 72, pose.getHeading() - Math.PI / 2);
    }

    public static Pose fromPedroPose(Pose pose) {
        return pose;
    }

    public void setPipeline(int id) {
        if (!limelight.isRunning())
            limelight.start();

        limelight.pipelineSwitch(id);
    }

    @Nullable
    public Pose getBotPose() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid())
            return null;

        Position robotPose = result.getBotpose().getPosition();
        return new Pose(robotPose.x * 100 / 2.54, robotPose.y * 100 / 2.54, Math.toRadians(0));
    }

    @Nullable
    public Pose getBotPose(double imuAngle) {
        limelight.updateRobotOrientation(imuAngle);
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid())
            return null;

        Position robotPose = result.getBotpose_MT2().getPosition();
        return new Pose(robotPose.x * 100 / 2.54, robotPose.y * 100 / 2.54, Math.toRadians(imuAngle));
    }
}
