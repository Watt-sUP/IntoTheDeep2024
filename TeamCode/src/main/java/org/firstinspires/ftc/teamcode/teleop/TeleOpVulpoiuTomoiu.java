package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

import java.util.concurrent.atomic.AtomicInteger;

@TeleOp(name = "TeleOp (Vulpoiu + Tomoiu)", group = "TeleOp")
public class TeleOpVulpoiuTomoiu extends TeleOpBase {
    @Override
    public void initialize() {
        super.initialize();

        /* Robot Face Integration */
        AtomicInteger robotFace = new AtomicInteger(1);

        chassis.setAxes(() -> driver1.getLeftY() * robotFace.get(), () -> driver1.getLeftX() * robotFace.get(), driver1::getRightX);
        driver1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> robotFace.set(robotFace.get() * -1));

        /* Brakes */
        driver1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> chassis.setMaxSpeed(0.5))
                .whenReleased(() -> chassis.setMaxSpeed(1));
        driver1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> chassis.setMaxSpeed(0.3))
                .whenReleased(() -> chassis.setMaxSpeed(1));

        /* Arm */
        driver2.getGamepadButton(GamepadKeys.Button.Y)
                .and(isTransferringTrigger)
                .whenActive(
                        new SequentialCommandGroup(
                                new ConditionalCommand(
                                        new InstantCommand(outtake::togglePivot),
                                        new InstantCommand(() -> outtake.setPivotState(OuttakeSubsystem.PivotState.SPECIMEN_DEPOSIT)),
                                        () -> outtake.getArmState() != OuttakeSubsystem.ArmState.SPECIMEN
                                ),
                                new InstantCommand(outtake::toggleArm)
                        )
                );
        driver2.getGamepadButton(GamepadKeys.Button.B)
                .and(isTransferringTrigger)
                .whenActive(outtake::toggleClaw);
    }
}
