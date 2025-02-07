package org.firstinspires.ftc.teamcode.autonomous.experiments;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.concurrent.atomic.AtomicInteger;

@Disabled
@Autonomous
public class AutoMenu extends CommandOpMode {
    public AtomicInteger yCoord = new AtomicInteger(67);
    public AtomicInteger extendoLength = new AtomicInteger(42);

    public void updateTelemetry() {
        telemetry.addData("Y Coordinate", yCoord.get() - 0.5);
        telemetry.addData("Extendo Length", extendoLength.get());
        telemetry.update();
    }

    public void updateY(int val) {
        int coord = yCoord.get() + val;
        if (coord >= 67 && coord <= 77) {
            yCoord.addAndGet(val);
        }
    }

    public void updateExt(int val) {
        int len = extendoLength.get() + val;
        if (len >= 0 && len <= 42) {
            extendoLength.addAndGet(val);
        }
    }

    @Override
    public void initialize() {
        GamepadEx driver1 = new GamepadEx(gamepad1);

        telemetry.setMsTransmissionInterval(10);

        while (opModeInInit()) {
            if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) updateY(1);
            if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) updateY(-1);
            if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) updateExt(1);
            if (driver1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) updateExt(-1);

            updateTelemetry();
            driver1.readButtons();
        }
    }
}
