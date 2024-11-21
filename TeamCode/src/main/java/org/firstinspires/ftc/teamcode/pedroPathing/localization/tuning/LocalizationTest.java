package org.firstinspires.ftc.teamcode.pedroPathing.localization.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot
 * on FTC Dashboard (192/168/43/1:8080/dash). You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/6/2024
 */
@Config
@TeleOp(group = "Pedro Pathing Tuning", name = "Localization Test")
public class LocalizationTest extends CommandOpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetryA;

    @Override
    public void initialize() {
        poseUpdater = new PoseUpdater(hardwareMap);

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        GamepadEx driver1 = new GamepadEx(gamepad1);

        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        drive.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        register(drive);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryA.update();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

        schedule(
                new RunCommand(() -> {
                    poseUpdater.update();
                    dashboardPoseTracker.update();

                    telemetryA.addData("x", poseUpdater.getPose().getX());
                    telemetryA.addData("y", poseUpdater.getPose().getY());
                    telemetryA.addData("heading", poseUpdater.getPose().getHeading());
                    telemetryA.addData("total heading", poseUpdater.getTotalHeading());
                    telemetryA.update();

                    Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
                    Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
                    Drawing.sendPacket();
                })
        );
    }
}
