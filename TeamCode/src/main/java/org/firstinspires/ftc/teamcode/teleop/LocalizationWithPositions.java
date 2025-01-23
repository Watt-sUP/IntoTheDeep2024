package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

import java.util.List;

@Disabled
@Config
@TeleOp(name = "Localization With Positions", group = "Utility")
public class LocalizationWithPositions extends CommandOpMode {
    public static boolean SPECIMEN_START = true;

    public static double OUTTAKE_ARM_POSITION = OuttakeSubsystem.ARM_SPECIMEN,
            OUTTAKE_ARM_PIVOT = OuttakeSubsystem.PIVOT_SPECIMEN_COLLECT;
    public static int OUTTAKE_SLIDES_POSITION = OuttakeSubsystem.SLIDES_LOWERED;

    @Override
    public void initialize() {
        Constants.setConstants(FConstants.class, LConstants.class);

        PoseUpdater poseUpdater = new PoseUpdater(hardwareMap);
        poseUpdater.setStartingPose(SPECIMEN_START ? AutonomousOpMode.START_POSE_SPECIMEN : AutonomousOpMode.START_POSE_BASKET);

        DashboardPoseTracker dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Trigger rightTrigger1 = new Trigger(() -> gamepad1.right_trigger >= 0.3);

        Trigger leftTrigger2 = new Trigger(() -> gamepad2.left_trigger >= 0.3);
        Trigger rightTrigger2 = new Trigger(() -> gamepad2.right_trigger >= 0.3);

        GamepadEx driver1 = new GamepadEx(gamepad1);
        GamepadEx driver2 = new GamepadEx(gamepad2);

        DriveSubsystem chassis = new DriveSubsystem(hardwareMap);

        chassis.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> chassis.setMaxSpeed(0.4))
                .whenReleased(() -> chassis.setMaxSpeed(1));
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> chassis.setMaxSpeed(0.6))
                .whenReleased(() -> chassis.setMaxSpeed(1));

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);

        /* Claw */
        leftTrigger2
                .whenActive(intake::togglePivot);
        driver2.getGamepadButton(GamepadKeys.Button.A)
                .whenActive(intake::toggleClaw);

        /* Claw Rotation */
        driver2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .and(new Trigger(() -> intake.getPivotState() != IntakeSubsystem.PivotState.UP))
                .whenActive(intake::nextRotation);
        driver2.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .and(new Trigger(() -> intake.getPivotState() != IntakeSubsystem.PivotState.UP))
                .whenActive(intake::previousRotation);

        rightTrigger1
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.EXTENDING)),
                        new ConditionalCommand(
                                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT)),
                                () -> intake.getExtendoState() != IntakeSubsystem.ExtendoState.IN
                        )
                ));

        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .whenActive(outtake::toggleClaw);

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        ElapsedTime runtime = new ElapsedTime();

        register(chassis, intake, outtake);

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

        schedule(
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new InstantCommand(() -> {
                            runtime.reset();

                            intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);
                            intake.setClawState(IntakeSubsystem.ClawState.OPENED);
                            intake.setPivotState(IntakeSubsystem.PivotState.UP);
                            intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT);

                            outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED);
                        }),
                        new RunCommand(() -> {
                            hubs.forEach(LynxModule::clearBulkCache);

                            outtake._setArmPosition(OUTTAKE_ARM_POSITION);
                            outtake._setPivotPosition(OUTTAKE_ARM_PIVOT);
                            outtake._setSlidesPosition(OUTTAKE_SLIDES_POSITION);

                            telemetry.addData("Status", "Running");
                            telemetry.addData("Runtime", "%.0f seconds", runtime.seconds());
                            telemetry.addLine();

                            telemetry.addData("Intake Extendo State", intake.getExtendoState().toString());
                            telemetry.addData("Intake Pivot State", intake.getPivotState().toString());
                            telemetry.addData("Intake Claw State", intake.getClawState().toString());
                            telemetry.addData("Intake Rotate State", intake.getRotation().toString());
                            telemetry.addLine();

                            telemetry.addData("Outtake Claw State", outtake.getClawState().toString());
                            telemetry.addData("Outtake Arm Position", outtake.getArmPosition());
                            telemetry.addData("Outtake Pivot Position", outtake.getPivotPosition());
                            telemetry.addLine();

                            telemetry.addData("Outtake Slides Position", outtake.getSlidesPosition());
                            telemetry.addData("Outtake Slides Target", outtake.getSlidesTarget());
                            telemetry.addData("Outtake Slides Current", "%.2f, %.2f", outtake.getSlidesCurrent()[0], outtake.getSlidesCurrent()[1]);
                            telemetry.addLine();

                            poseUpdater.update();
                            dashboardPoseTracker.update();

                            telemetry.addData("X", poseUpdater.getPose().getX());
                            telemetry.addData("Y", poseUpdater.getPose().getY());
                            telemetry.addData("Heading", poseUpdater.getPose().getHeading());
                            telemetry.addData("Total Heading", poseUpdater.getTotalHeading());

                            Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
                            Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
                            Drawing.sendPacket();

                            telemetry.update();
                        })
                )
        );
    }
}
