package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class AutonomousOpMode extends CommandOpMode {
    public static final Pose START_POSE_SPECIMEN = new Pose(7.95, 66, Math.toRadians(180));
    public static final Pose START_POSE_BASKET = new Pose(6.5, 104, Math.toRadians(270));

    public static Follower follower;

    public static IntakeSubsystem intake;
    public static OuttakeSubsystem outtake;

    @Override
    public void initialize() {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Constants.setConstants(FConstants.class, LConstants.class);

        follower = new Follower(hardwareMap);

        intake = new IntakeSubsystem(hardwareMap);
        outtake = new OuttakeSubsystem(hardwareMap);

        register(intake, outtake);

        schedule(
                new RunCommand(() -> {
                    follower.update();
                    try {
                        follower.telemetryDebug(telemetry);
                    } catch (RuntimeException ignored) {
                    }
                })
        );
    }

    public void enableInit() {
        schedule(
                new InstantCommand(() -> {
                    intake.setPivotState(IntakeSubsystem.PivotState.UP);
                    intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT);
                    intake.setClawState(IntakeSubsystem.ClawState.OPENED);

                    outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED);

                    outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER);
                    outtake._setArmPosition(45);

                    outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                    outtake._setPivotPosition(0.92);
                })
        );
    }

    public void startSpecimen() {
        follower.setStartingPose(START_POSE_SPECIMEN);
    }

    public void startBasket() {
        follower.setStartingPose(START_POSE_BASKET);
    }
}
