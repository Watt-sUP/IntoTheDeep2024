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

        SequentialCommandGroup transferCommand = new SequentialCommandGroup(
                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(250),
                new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED)),
                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.TRANSFER)),
                new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.IN)),
                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),
                new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT)),
                new WaitCommand(75),
                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                new WaitCommand(300),
                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
                new WaitCommand(150),
                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),
                new WaitCommand(100),
                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.OUT)),
                new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.OUT)),
                new WaitCommand(100),
                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.COLLECT))
        );

        SequentialCommandGroup depositBasketCommand = new SequentialCommandGroup(
                new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET)),
                new WaitCommand(1000),
                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.OUT)),
                new WaitCommand(500),
                new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED)),
                new WaitCommand(300),
                new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.IN)),
                new WaitCommand(500),
                new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED)),
                new WaitCommand(300)
        );

        schedule(
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),

                        new FollowPointCommand(follower, Basket.getPreloadBasketPose(), 1)
                                .andThen(new WaitCommand(1000)),
                        depositBasketCommand/*,

                        new FollowPointCommand(follower, SpikeYellowSamples.LEFT.getPose(), 1)
                                .alongWith(
                                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.HORIZONTAL))
                                ),

                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.HALF)),
                                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),
                                new WaitCommand(300),
                                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.DOWN)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED))
                        ),
                        new WaitCommand(500),

                        // Go to basket
                        new FollowPointCommand(follower, Basket.getBasketPose(), 1)
                                .alongWith(transferCommand),
                        depositBasketCommand,

                        // Go to the middle sample
                        new FollowPointCommand(follower, SpikeYellowSamples.MIDDLE.getPose())
                                .alongWith(
                                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.HORIZONTAL))
                                ),

                        // Pick up sample #2
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.HALF)),
                                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),
                                new WaitCommand(300),
                                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.DOWN)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED))
                        ),
                        new WaitCommand(500),

                        // Go to basket
                        new FollowPointCommand(follower, Basket.getBasketPose(), 1)
                                .alongWith(transferCommand),
                       depositBasketCommand,

                        // Go to the right sample
                        new FollowPointCommand(follower, SpikeYellowSamples.RIGHT.getPose())
                                .alongWith(
                                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT))
                                ),

                        // Pick up sample #3
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.HALF)),
                                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),
                                new WaitCommand(300),
                                new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.DOWN)),
                                new WaitCommand(1000),
                                new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.CLOSED))
                        ),
                        new WaitCommand(500),

                        // Go to basket
                        new FollowPointCommand(follower, SpikeYellowSamples.RIGHT.getPose())
                                .alongWith(transferCommand),
                        depositBasketCommand*/
                )
        );
    }
}