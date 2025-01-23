package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

import javax.annotation.Nullable;

/**
 * <p>This class handles Limelight detections.
 * All detections are made in the <a href="https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html">global coordinate system</a>.</p>
 * Use {@link #toPedroPose(Pose)} or {@link #toPedroPoseNeutral(Pose)} for conversions to the Pedro system.
 */
public class LimelightSubsystem extends SubsystemBase {

    private final Limelight3A limelight;
    private boolean revertHeading = false;

    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    /**
     * Converts a pose from the <a href="https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html">global coordinate system</a> to the Pedro one.
     *
     * @param pose A pose/detection in the FTC coordinate system
     * @return A pose in the Pedro Pathing coordinate system
     */
    @NonNull
    public static Pose toPedroPose(Pose pose) {
        return new Pose(Math.abs(pose.getY() - 72), pose.getX() + 72, pose.getHeading() + Math.PI / 2);
    }

    /**
     * Converts a Pedro Pathing pose to the global coordinate system.
     *
     * @param pose A pose in the Pedro Pathing coordinate system
     * @return A pose in the FTC coordinate system
     */
    @NonNull
    public static Pose fromPedroPose(Pose pose) {
        return new Pose(pose.getY() - 72, pose.getX() - 72, pose.getHeading() - Math.PI / 2);
    }

    /**
     * Converts a pose from the <a href="https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html">global coordinate system</a> to an alliance neutral version of the Pedro Pathing one.
     *
     * @param pose A pose/detection in the FTC coordinate system
     * @return A normalized pose in the Pedro Pathing coordinate system (alliance-specific)
     */
    @NonNull
    public static Pose toPedroPoseNeutral(Pose pose) {
        Pose rawPedro = toPedroPose(pose);

        if (rawPedro.getX() <= 72)
            return rawPedro;
        else return new Pose(144 - rawPedro.getX(),
                144 - rawPedro.getY(), rawPedro.getHeading() + Math.PI);
    }

    public void setPipeline(int id) {
        if (!limelight.isRunning())
            limelight.start();

        limelight.pipelineSwitch(id);
    }

    /**
     * Gets the robot's pose using AprilTags exclusively
     *
     * @return Limelight MegaTag1 pose
     */
    @Nullable
    public Pose getBotPose() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid())
            return null;

        Position robotPose = result.getBotpose().getPosition();
        return new Pose(robotPose.x * 100 / 2.54, robotPose.y * 100 / 2.54, Math.toRadians(0));
    }

    /**
     * Fuses the AprilTag detections with the robot's IMU angle for more accurate readings.
     * This also avoids the problem of tag-flipping.
     *
     * @param imuAngle The robot's current angle in the global field system (in degrees)
     * @return Limelight MegaTag2 pose
     */
    @Nullable
    public Pose getBotPose(double imuAngle) {
        limelight.updateRobotOrientation(imuAngle + (revertHeading ? 180 : 0));
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid())
            return null;

        Position robotPose = result.getBotpose_MT2().getPosition();
        Pose output = new Pose(robotPose.x * 100 / 2.54, robotPose.y * 100 / 2.54, Math.toRadians(imuAngle));

        // TODO: Check if this approach works. If it fails when very close to edges, add a buffer/threshold
        if (Range.clip(output.getX(), -72, 72) != output.getX() || Range.clip(output.getY(), -72, 72) != output.getY()) {
            revertHeading = true;
            return getBotPose(imuAngle);
        }

        return output;
    }
}
