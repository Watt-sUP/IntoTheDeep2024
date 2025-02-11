package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.util.Drawing;

public class AutonomousOpMode extends CommandOpMode {
    public static final Pose2D START_POSE_SPECIMEN = new Pose2D(
            DistanceUnit.INCH,
            7.95,
            66,
            AngleUnit.DEGREES,
            180
    );
    public static final Pose2D START_POSE_BASKET = new Pose2D(
            DistanceUnit.INCH,
            6.5,
            104,
            AngleUnit.DEGREES,
            270
    );

    public static IntakeSubsystem intake;
    public static OuttakeSubsystem outtake;
    public static DriveSubsystem drive;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);

        drive.setTeleOpMode(false);
        drive.setMotorBehavior(Motor.ZeroPowerBehavior.BRAKE);
        drive.setPosition(0, 0, 0);

        DashboardPoseTracker poseTracker = new DashboardPoseTracker(drive);

        register(intake, outtake, drive);

        schedule(
                new RunCommand(() -> {
                    Pose2D currentPosition = drive.getPosition();
                    Pose2D targetPosition = drive.getTarget();
                    telemetry.addData("Current Position",
                            "X: %.2f in, Y: %.2f in, H: %.2f deg",
                            currentPosition.getX(DistanceUnit.INCH),
                            currentPosition.getY(DistanceUnit.INCH),
                            currentPosition.getHeading(AngleUnit.DEGREES)
                    );
                    telemetry.addData("Target Position",
                            "X: %.2f in, Y: %.2f in, H: %.2f deg",
                            targetPosition.getX(DistanceUnit.INCH),
                            targetPosition.getY(DistanceUnit.INCH),
                            targetPosition.getHeading(AngleUnit.DEGREES)
                    );
                    telemetry.addData("At Target", drive.atTarget() ? "Yes" : "No");
                    telemetry.addLine();

                    telemetry.addData("X Error (inches)", drive.getXError(DistanceUnit.INCH));
                    telemetry.addData("Y Error (inches)", drive.getYError(DistanceUnit.INCH));
                    telemetry.addData("Total Distance Error (inches)", drive.getDistanceError(DistanceUnit.INCH));
                    telemetry.addData("Heading Error (degrees)", drive.getHeadingError(AngleUnit.DEGREES));
                    telemetry.addLine();

                    telemetry.addData("Max Speed", "%d%", drive.getMaxSpeed());

                    Drawing.drawPoseHistory(poseTracker, "#4CAF50");
                    Drawing.drawRobot(currentPosition, "#4CAF50");
                    Drawing.drawRobot(targetPosition, "#3F51B5");
                    Drawing.sendPacket();

                    telemetry.update();
                })
        );
    }

    public void enableInit() {
        schedule(
                new InstantCommand(() -> {
                    intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);
                    intake.setPivotState(IntakeSubsystem.PivotState.UP);
                    intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT);
                    intake.setClawState(IntakeSubsystem.ClawState.OPENED);

                    outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED);

                    outtake.setArmState(OuttakeSubsystem.ArmState.IN);
                    outtake._setArmPosition(25);

                    outtake.setPivotState(OuttakeSubsystem.PivotState.OUT);
                    outtake._setPivotPosition(0);

                    outtake.resetSlidesEncoder();
                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                })
        );
    }

    public void startSpecimen() {
        drive.setPosition(START_POSE_SPECIMEN);
    }

    public void startBasket() {
        drive.setPosition(START_POSE_BASKET);
    }
}
