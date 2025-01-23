package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.commands.FollowPointCommand;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

@Disabled
@Autonomous(name = "Basket Autonomous", group = "Auto Experiments")
public class BasketAuto extends AutonomousOpMode {
    public static final Pose BASKET_POSE = new Pose(18, 122, Math.toRadians(315));

    @Override
    public void initialize() {
        super.initialize();
        startBasket();
        enableInit();

        schedule(
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),

                        new FollowPointCommand(follower, BASKET_POSE, 1)
                                .alongWith(
                                        new SequentialCommandGroup(
                                                new WaitCommand(250),
                                                new InstantCommand(() -> {
                                                    outtake.setSlidesState(OuttakeSubsystem.SlidesState.HIGH_BASKET);
                                                    outtake.setArmState(OuttakeSubsystem.ArmState.OUT);
                                                    outtake.setPivotState(OuttakeSubsystem.PivotState.OUT);
                                                    new WaitCommand(1750);
                                                })
                                        )
                                ),

                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                                .alongWith(new WaitCommand(250)),

                        new InstantCommand(() -> {
                            outtake.setArmState(OuttakeSubsystem.ArmState.IN);
                            outtake.setPivotState(OuttakeSubsystem.PivotState.IN);
                        }).alongWith(new WaitCommand(200)),
                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.LOWERED))
                )
        );
    }
}
