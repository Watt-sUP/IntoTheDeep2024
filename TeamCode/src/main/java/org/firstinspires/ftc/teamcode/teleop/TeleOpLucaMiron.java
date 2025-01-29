package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeNewPID;

@TeleOp(name = "TeleOp (Luca + Miron)", group = "TeleOp")
public class TeleOpLucaMiron extends TeleOpBase {
    @Override
    public void initialize() {
        super.initialize();

        /* Brakes */
        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> chassis.setMaxSpeed(0.4))
                .whenReleased(() -> chassis.setMaxSpeed(1));
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> chassis.setMaxSpeed(0.6))
                .whenReleased(() -> chassis.setMaxSpeed(1));

        /* Arm */
        driver2.getGamepadButton(GamepadKeys.Button.B)
                .and(isTransferringTrigger)
                .whenActive(
                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        new InstantCommand(outtake::togglePivot),
                                        new InstantCommand(() -> outtake.setPivotState(OuttakeNewPID.PivotState.SPECIMEN_DEPOSIT)),
                                        () -> outtake.getArmState() != OuttakeNewPID.ArmState.SPECIMEN
                                ),
                                new InstantCommand(outtake::toggleArm)
                        )
                );
        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .and(isTransferringTrigger)
                .whenActive(outtake::toggleClaw);
    }
}
