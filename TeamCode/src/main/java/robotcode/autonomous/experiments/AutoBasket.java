package robotcode.autonomous.experiments;

import static robotcode.autonomous.assets.AutonomousConstants.START_POSE_BASKET;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import robotcode.autonomous.assets.AutonomousCommandOpMode;
import robotcode.autonomous.assets.Submersible;
import robotcode.commands.FollowPathCommand;
import robotcode.pedroPathing.constants.FConstants;
import robotcode.pedroPathing.constants.LConstants;
import robotcode.subsystems.IntakeSubsystem;
import robotcode.subsystems.OuttakeSubsystem;
import robotcode.util.FixedSequentialCommandGroup;

@Disabled
@Autonomous(name = "Basket Autonomous", group = "Auto Experiments")
public class AutoBasket extends AutonomousCommandOpMode {
    @Override
    public void initialize() {
        Constants.setConstants(FConstants.class, LConstants.class);
        
        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(START_POSE_BASKET);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        OuttakeSubsystem outtake = new OuttakeSubsystem(hardwareMap);

        register(intake, outtake);

        scheduleOnRun(
                new RunCommand(follower::update),
                new RunCommand(() -> follower.telemetryDebug(telemetry)),
                new FixedSequentialCommandGroup(
                        new InstantCommand(() -> intake.setExtendoState(IntakeSubsystem.ExtendoState.IN)),
                        new InstantCommand(() -> intake.setPivotState(IntakeSubsystem.PivotState.UP)),
                        new InstantCommand(() -> intake.setRotation(IntakeSubsystem.RotationState.STRAIGHT)),
                        new InstantCommand(() -> intake.setClawState(IntakeSubsystem.ClawState.OPENED)),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.CLOSED)),
                        new InstantCommand(() -> outtake.setArmState(OuttakeSubsystem.ArmState.OUT)),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_COLLECT)),
                        new InstantCommand(() -> follower.setMaxPower(0.6)),
                        new FollowPathCommand(follower, Submersible.startPathSpecimen).alongWith(
                                new SequentialCommandGroup(
                                        new WaitCommand(350),
                                        new InstantCommand(() -> outtake.setSlidesState(OuttakeSubsystem.SlidesState.SPECIMEN)),
                                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.OUT))
                                )
                        ).withTimeout(5000),
                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT)),
                        new WaitCommand(450),
                        new InstantCommand(() -> follower.setMaxPower(1)),
                        new FollowPathCommand(follower, Submersible.depositPath(0)).setHoldEnd(false).withTimeout(3000),
                        new InstantCommand(() -> outtake.setClawState(OuttakeSubsystem.ClawState.OPENED))
                )
        );
    }
}
