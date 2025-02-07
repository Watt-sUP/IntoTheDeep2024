package org.firstinspires.ftc.teamcode.autonomous.experiments;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.autonomous.assets.Basket;
import org.firstinspires.ftc.teamcode.autonomous.assets.SpikeYellowSamples;
import org.firstinspires.ftc.teamcode.autonomous.assets.Submersible;
import org.firstinspires.ftc.teamcode.commands.FollowPointCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

@Config
@Autonomous
public class NewBasketAuto extends AutonomousOpMode {
    public static double MAX_EXT_LENGTH = 15.5;

    public AtomicInteger xCoord = new AtomicInteger(58);
    public AtomicInteger extendoLength = new AtomicInteger(1);
    public AtomicReference<IntakeSubsystem.RotationState> clawRotation = new AtomicReference<>(IntakeSubsystem.RotationState.STRAIGHT);

    private AtomicInteger currentSample = new AtomicInteger(0);

    public Command collectAndDeposit(SpikeYellowSamples sample) {
        int pos = currentSample.incrementAndGet();

        return new FixedSequentialCommandGroup(
                new FollowPointCommand(follower, sample.POSE, 0.6)
                        .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new InstantCommand(() -> {
                                            outtake.setArmState(OuttakeSubsystem.ArmState.IN);
                                            outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                                            outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                                        })
                                )
                        ),

                new WaitCommand(175),

                new InstantCommand(() -> {
                    intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT);
                    intake.setPivotState(IntakeSubsystem.PivotState.COLLECT);
                }).andThen(new WaitCommand(450)),

                new ConditionalCommand(
                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.HORIZONTAL))
                                .andThen(new WaitCommand(150)),
                        new WaitCommand(0),
                        () -> sample == SpikeYellowSamples.RIGHT
                ),

                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.DOWN)),
                new WaitCommand(125),
                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(175),
                new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT)),
                new WaitCommand(150),
                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                new WaitCommand(450),
                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER)),
                new WaitCommand(125),
                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),

                new FollowPointCommand(follower, Basket.getPose(pos), 1)
                        .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)),
                                        new WaitCommand(1000),
                                        new InstantCommand(() -> {
                                            outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                                            outtake.setPivotState(OuttakeSubsystem.PivotState.OUT);
                                        }),
                                        new WaitCommand(750)
                                )
                        ),

                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                        .andThen(new WaitCommand(200))
        );
    }

    public void updateTelemetry() {
        telemetry.addData("X Coordinate", xCoord.get());
        telemetry.addData("Extendo Length", extendoLength.get());
        telemetry.addData("Claw Rotation", clawRotation.get());
        telemetry.update();
    }

    public void updateX(int val) {
        int coord = xCoord.get() + val;
        if (coord >= 58 && coord <= 66) {
            xCoord.addAndGet(val);
        }
    }

    public void updateExt(int val) {
        int len = extendoLength.get() + val;
        if (len >= 0 && len <= MAX_EXT_LENGTH) {
            extendoLength.addAndGet(val);
        }
    }

    @Override
    public void initialize() {
        super.initialize();
        startBasket();
        enableInit();
//        enableLimelight();

        GamepadEx driver1 = new GamepadEx(gamepad1);

        telemetry.setMsTransmissionInterval(10);

        while (opModeInInit()) {
            if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) updateX(1);
            if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) updateX(-1);
            if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) updateExt(1);
            if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) updateExt(-1);

            if (driver1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON))
                clawRotation.updateAndGet(IntakeSubsystem.RotationState::next);
            if (driver1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON))
                clawRotation.updateAndGet(IntakeSubsystem.RotationState::previous);

            updateTelemetry();
            driver1.readButtons();
        }

        schedule(
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),

                        new FollowPointCommand(follower, Basket.getPose(0), 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(250),
                                                new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)),
                                                new WaitCommand(1000),
                                                new InstantCommand(() -> {
                                                    outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                                                    outtake.setPivotState(OuttakeSubsystem.PivotState.OUT);
                                                }),
                                                new WaitCommand(650)
                                        )
                                ),

                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                                .andThen(new WaitCommand(200)),

                        collectAndDeposit(SpikeYellowSamples.LEFT),
                        collectAndDeposit(SpikeYellowSamples.MIDDLE),
                        collectAndDeposit(SpikeYellowSamples.RIGHT),

                        new FollowPointCommand(follower, new Pose(xCoord.get(), 106, Math.toRadians(270)), 12)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(250),
                                                new InstantCommand(() -> {
                                                    outtake.setArmState(OuttakeSubsystem.ArmState.IN);
                                                    outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                                                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                                                })
                                        )
                                ),
                        new FollowPointCommand(follower, new Pose(xCoord.get(), 95.2, Math.toRadians(270)), 0.5),

                        new InstantCommand(() -> {
                            intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT);
                            intake._setExtendoPosition(extendoLength.get() / MAX_EXT_LENGTH);

                            intake.setPivotState(IntakeSubsystem.PivotState.COLLECT);
                        }).andThen(new WaitCommand(500)),

                        new ConditionalCommand(
                                new InstantCommand(() -> intake.setRotation(clawRotation.get()))
                                        .andThen(new WaitCommand(350)),
                                new WaitCommand(0),
                                () -> clawRotation.get() != IntakeSubsystem.RotationState.STRAIGHT
                        ),

                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.DOWN)),
                        new WaitCommand(175),
                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(175),
                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT)),
                        new WaitCommand(175),
                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
                        new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                        new WaitCommand(450),

                        new FollowPointCommand(follower, Basket.getPose(currentSample.incrementAndGet()), 1.3)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER)),
                                                new WaitCommand(150),
                                                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                                                new WaitCommand(100),
                                                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),
                                                new WaitCommand(250),
                                                new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)),
                                                new WaitCommand(1000),
                                                new InstantCommand(() -> {
                                                    outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                                                    outtake.setPivotState(OuttakeSubsystem.PivotState.OUT);
                                                }),
                                                new WaitCommand(750)
                                        )
                                ),

                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                                .andThen(new WaitCommand(200)),

                        new InstantCommand(() -> follower.setMaxPower(0.9)),

                        new FollowPointCommand(follower, Submersible.ascentParkPose, 5)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(350),
                                                new InstantCommand(() -> outtake._setArmPosition(175)),
                                                new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED))
                                        )
                                ),

                        new InstantCommand(this::requestOpModeStop)
                )
        );
    }
}
