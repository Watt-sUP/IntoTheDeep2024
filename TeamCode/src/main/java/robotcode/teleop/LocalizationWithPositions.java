package robotcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import robotcode.autonomous.assets.AutonomousConstants;
import robotcode.subsystems.DriveSubsystem;
import robotcode.subsystems.IntakeSubsystem;
import robotcode.subsystems.OuttakeSubsystem;
import robotcode.util.FixedSequentialCommandGroup;

@Config
@TeleOp(name = "Localization With Positions", group = "Utility")
public class LocalizationWithPositions extends CommandOpMode {
    public static boolean SPECIMEN_START = true;

    public static IntakeSubsystem.ExtendoState INTAKE_EXTENDO_STATE = IntakeSubsystem.ExtendoState.IN;
    public static IntakeSubsystem.PivotState INTAKE_PIVOT_STATE = IntakeSubsystem.PivotState.UP;
    public static IntakeSubsystem.ClawState INTAKE_CLAW_STATE = IntakeSubsystem.ClawState.OPENED;
    public static IntakeSubsystem.RotationState INTAKE_ROTATION_STATE = IntakeSubsystem.RotationState.STRAIGHT;

    public static double OUTTAKE_ARM_POSITION = OuttakeSubsystem.ARM_SPECIMEN,
            OUTTAKE_ARM_PIVOT = OuttakeSubsystem.PIVOT_SPECIMEN_COLLECT;
    public static int OUTTAKE_SLIDES_POSITION = OuttakeSubsystem.SLIDES_LOWERED;
    public static OuttakeSubsystem.ClawState OUTTAKE_CLAW_STATE = OuttakeSubsystem.ClawState.CLOSED;

    @Override
    public void initialize() {
        this.reset();

        Constants.setConstants(FConstants.class, LConstants.class);

        PoseUpdater poseUpdater = new PoseUpdater(hardwareMap);
        poseUpdater.setStartingPose(SPECIMEN_START ? AutonomousConstants.START_POSE_SPECIMEN : AutonomousConstants.START_POSE_BASKET);

        DashboardPoseTracker dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamepadEx driver1 = new GamepadEx(gamepad1);

        DriveSubsystem chassis = new DriveSubsystem(hardwareMap);

        chassis.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> chassis.setMaxSpeed(0.4))
                .whenReleased(() -> chassis.setMaxSpeed(1));
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> chassis.setMaxSpeed(0.6))
                .whenReleased(() -> chassis.setMaxSpeed(1));

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        ElapsedTime runtime = new ElapsedTime();

        register(intake, outtake);

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

        schedule(
                new InstantCommand(() -> {
                    telemetry.addData("Status", "Initialized");
                    telemetry.update();
                }),
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        new InstantCommand(runtime::reset),
                        new RunCommand(() -> hubs.forEach(LynxModule::clearBulkCache)),
                        new RunCommand(() -> {
                            intake.setExtendoState(INTAKE_EXTENDO_STATE);
                            intake.setPivotState(INTAKE_PIVOT_STATE);
                            intake.setClawState(INTAKE_CLAW_STATE);
                            intake.setRotation(INTAKE_ROTATION_STATE);

                            outtake.setClawState(OUTTAKE_CLAW_STATE);
                            outtake._setArmPosition(OUTTAKE_ARM_POSITION);
                            outtake._setPivotPosition(OUTTAKE_ARM_PIVOT);
                            outtake._setSlidesPosition(OUTTAKE_SLIDES_POSITION);
                        }),
                        new RunCommand(() -> {
                            telemetry.addData("Status", "Running");
                            telemetry.addData("Runtime", "%.0f seconds", runtime.seconds());
                            telemetry.addLine();

                            telemetry.addData("Intake Extendo State", intake.getExtendoState().toString());
                            telemetry.addData("Intake Pivot State", intake.getPivotState().toString());
                            telemetry.addData("Intake Claw State", intake.getClawState().toString());
                            telemetry.addData("Intake Rotate State", intake.getRotation().toString());
                            telemetry.addLine();

                            telemetry.addData("Outtake Claw State", outtake.getClawState().toString());
                            telemetry.addData("Outtake Arm Position", outtake.getArmState().toString());
                            telemetry.addData("Outtake Pivot Position", outtake.getPivotState().toString());
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
