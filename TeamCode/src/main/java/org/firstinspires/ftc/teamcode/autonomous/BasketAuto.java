package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.autonomous.assets.Basket;
import org.firstinspires.ftc.teamcode.autonomous.assets.SpikeYellowSamples;
import org.firstinspires.ftc.teamcode.commands.FollowPointCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

import java.util.concurrent.atomic.AtomicInteger;

@Autonomous(name = "Basket Autonomous", group = "Autonomous")
public class BasketAuto extends AutonomousOpMode {
    private AtomicInteger currentSample = new AtomicInteger(0);

    public Command collectAndDeposit(SpikeYellowSamples sample) {
        int pos = currentSample.incrementAndGet();

        return new FixedSequentialCommandGroup(
                new FollowPointCommand(follower, sample.getPose(), 1)
                        .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new InstantCommand(() -> {
                                            outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER);
                                            outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                                            outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED);
                                        })
                                )
                        ),

                new WaitCommand(500),

                new InstantCommand(() -> {
                    intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT);
                    intake.setPivotState(IntakeSubsystem.PivotState.DOWN);
                }).andThen(new WaitCommand(1000)),

                new ConditionalCommand(
                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.LEFT)),
                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT)),
                        () -> sample == SpikeYellowSamples.RIGHT
                ).andThen(new WaitCommand(250)),

                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(250),
                new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT)),
                new WaitCommand(150),
                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                new WaitCommand(650),
                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
                new WaitCommand(150),
                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),

                new FollowPointCommand(follower, Basket.getBasketPose(pos), 1)
                        .alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(250),
                                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)),
                                        new WaitCommand(1000),
                                        new InstantCommand(() -> {
                                            outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                                            outtake.setPivotState(OuttakeSubsystem.PivotState.OUT);
                                        }),
                                        new WaitCommand(700)
                                )
                        ),

                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                        .andThen(new WaitCommand(300))
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        startBasket();
        enableInit();

        schedule(
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),

                        // Preload

                        new FollowPointCommand(follower, Basket.getBasketPose(0), 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(250),
                                                new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)),
                                                new WaitCommand(1000),
                                                new InstantCommand(() -> {
                                                    outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                                                    outtake.setPivotState(OuttakeSubsystem.PivotState.OUT);
                                                }),
                                                new WaitCommand(700)
                                        )
                                ),

                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                                .andThen(new WaitCommand(300)),

                        collectAndDeposit(SpikeYellowSamples.LEFT),
                        collectAndDeposit(SpikeYellowSamples.MIDDLE),
                        collectAndDeposit(SpikeYellowSamples.RIGHT)
                )
        );
    }
}