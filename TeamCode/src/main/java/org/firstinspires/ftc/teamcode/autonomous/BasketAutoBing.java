package org.firstinspires.ftc.teamcode.autonomous;

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

@Autonomous(name = "Basket Auto Bing", group = "Autonomous")
public class BasketAutoBing extends AutonomousOpMode {
    @Override
    public void initialize() {
        super.initialize();
        startBasket();
        enableInit();

        schedule(
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),

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


                        new FollowPointCommand(follower, SpikeYellowSamples.LEFT.getPose(), 1)
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

                        new WaitCommand(300),

                        new InstantCommand(() -> {
                            intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT);
                            intake.setPivotState(IntakeSubsystem.PivotState.DOWN);
                        }).andThen(new WaitCommand(500)),

                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(250),
                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
                        new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                        new WaitCommand(300),
                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
                        new WaitCommand(150),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(100),
                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),

                        // Go to basket
                        new FollowPointCommand(follower, Basket.getBasketPose(1), 1)
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

                        // Go to the middle sample
                        new FollowPointCommand(follower, SpikeYellowSamples.MIDDLE.getPose())
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

                        new WaitCommand(300),

                        new InstantCommand(() -> {
                            intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT);
                            intake.setPivotState(IntakeSubsystem.PivotState.DOWN);
                        }).andThen(new WaitCommand(500)),

                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(250),
                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
                        new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                        new WaitCommand(300),
                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
                        new WaitCommand(150),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(100),
                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),

                        // Go to basket
                        new FollowPointCommand(follower, Basket.getBasketPose(2), 1)
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

                        // Go to the right sample
                        new FollowPointCommand(follower, SpikeYellowSamples.RIGHT.getPose())
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

                        new WaitCommand(300),

                        new InstantCommand(() -> {
                            intake.setExtendoState(IntakeSubsystem.ExtendoState.OUT);
                            intake.setPivotState(IntakeSubsystem.PivotState.DOWN);
                        }).andThen(new WaitCommand(500)),

                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.LEFT)),

                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(250),
                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
                        new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                        new WaitCommand(300),
                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
                        new WaitCommand(150),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                        new WaitCommand(100),
                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),

                        new FollowPointCommand(follower, Basket.getBasketPose(3), 1)
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
                )
        );
    }
}