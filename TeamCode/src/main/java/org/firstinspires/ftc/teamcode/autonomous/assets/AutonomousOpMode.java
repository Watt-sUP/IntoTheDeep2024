package org.firstinspires.ftc.teamcode.autonomous.assets;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

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


        register(intake, outtake, drive);

        schedule(
                new RunCommand(() -> {

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

//    public void startSpecimen() {
//        drive.setPosition(START_POSE_SPECIMEN);
//    }
//
//    public void startBasket() {
//        drive.setPosition(START_POSE_BASKET);
//    }
}
