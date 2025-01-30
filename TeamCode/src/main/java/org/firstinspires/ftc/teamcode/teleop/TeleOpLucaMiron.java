package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

@TeleOp(name = "TeleOp (Luca + Miron)", group = "TeleOp")
public class TeleOpLucaMiron extends TeleOpBase {
    @Override
    public void initialize() {
        super.initialize();
        setBrakes(0.4, 0.6);

        /* Arm */
        driver2.getGamepadButton(GamepadKeys.Button.B)
                .and(isTransferringTrigger)
                .whenActive(
                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        new InstantCommand(outtake::togglePivot),
                                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT)),
                                        () -> outtake.getArmState() != OuttakeSubsystem.ArmState.SPECIMEN_COLLECT
                                ),
                                new InstantCommand(outtake::toggleArm)
                        )
                );
        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .and(isTransferringTrigger)
                .whenActive(outtake::toggleClaw);
    }
}
