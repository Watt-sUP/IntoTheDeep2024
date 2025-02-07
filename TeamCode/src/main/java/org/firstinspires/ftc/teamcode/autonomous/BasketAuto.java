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
import org.firstinspires.ftc.teamcode.autonomous.assets.Submersible;
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

                new WaitCommand(350),

                new InstantCommand(() -> {
                    intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT);
                    intake.setPivotState(IntakeSubsystem.PivotState.COLLECT);
                }).andThen(new WaitCommand(450)),

                new ConditionalCommand(
                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.HORIZONTAL))
                                .andThen(new WaitCommand(100)),
                        new WaitCommand(0),
                        () -> sample == SpikeYellowSamples.RIGHT
                ),

                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.DOWN)),
                new WaitCommand(175),
                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(225),
                new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT)),
                new WaitCommand(250),
                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                new WaitCommand(450),
                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER)),
                new WaitCommand(150),
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
                        .andThen(new WaitCommand(300))
        );
    }

    @Override
    public void initialize() {
        super.initialize();
        startBasket();
        enableInit();
//        enableLimelight();

        schedule(
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),

                        // Preload

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
                                                new WaitCommand(850)
                                        )
                                ),

                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                                .andThen(new WaitCommand(300)),

                        collectAndDeposit(SpikeYellowSamples.LEFT),
                        collectAndDeposit(SpikeYellowSamples.MIDDLE),
                        collectAndDeposit(SpikeYellowSamples.RIGHT),

                        new InstantCommand(() -> follower.setMaxPower(0.8)),

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