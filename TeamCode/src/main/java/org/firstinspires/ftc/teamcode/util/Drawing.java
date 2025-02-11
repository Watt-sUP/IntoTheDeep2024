package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * This is the Drawing class. It handles the drawing of stuff on FTC Dashboard, like the robot.
 *
 * @author Logan Nash
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 4/22/2024
 */
public class Drawing {
    public static final double ROBOT_RADIUS = 9;

    private static TelemetryPacket packet;


    /**
     * This adds instructions to the current packet to draw a robot at a specified Pose with a specified
     * color. If no packet exists, then a new one is created.
     *
     * @param pose  the Pose to draw the robot at
     * @param color the color to draw the robot with
     */
    public static void drawRobot(Pose2D pose, String color) {
        if (packet == null) packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke(color);
        Drawing.drawRobotOnCanvas(packet.fieldOverlay(), pose);
    }

    /**
     * This adds instructions to the current packet to draw the pose history of the robot. If no
     * packet exists, then a new one is created.
     *
     * @param poseTracker the DashboardPoseTracker to get the pose history from
     * @param color       the color to draw the pose history with
     */
    public static void drawPoseHistory(DashboardPoseTracker poseTracker, String color) {
        if (packet == null) packet = new TelemetryPacket();

        packet.fieldOverlay().setStroke(color);
        packet.fieldOverlay().strokePolyline(poseTracker.getXPositionsArray(), poseTracker.getYPositionsArray());
    }

    /**
     * This tries to send the current packet to FTC Dashboard.
     *
     * @return returns if the operation was successful.
     */
    public static boolean sendPacket() {
        if (packet != null) {
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            packet = null;
            return true;
        }
        return false;
    }

    /**
     * This draws a robot on the Dashboard at a specified Pose. This is more useful for drawing the
     * actual robot, since the Pose contains the direction the robot is facing as well as its position.
     *
     * @param c the Canvas on the Dashboard on which this will draw at
     * @param t the Pose to draw at
     */
    public static void drawRobotOnCanvas(Canvas c, Pose2D t) {
        c.strokeCircle(t.getX(DistanceUnit.INCH), t.getY(DistanceUnit.INCH), ROBOT_RADIUS);
        double x = ROBOT_RADIUS * Math.cos(t.getHeading(AngleUnit.RADIANS));
        double y = ROBOT_RADIUS * Math.sin(t.getHeading(AngleUnit.RADIANS));
        double x1 = t.getX(DistanceUnit.INCH) + x / 2, y1 = t.getY(DistanceUnit.INCH) + y / 2;
        double x2 = t.getX(DistanceUnit.INCH) + x, y2 = t.getY(DistanceUnit.INCH) + y;
        c.strokeLine(x1, y1, x2, y2);
    }

    /**
     * This draws a Path on the Dashboard from a specified Array of Points.
     *
     * @param c      the Canvas on the Dashboard on which this will draw
     * @param points the Points to draw
     */
    public static void drawPath(Canvas c, double[][] points) {
        c.strokePolyline(points[0], points[1]);
    }
}