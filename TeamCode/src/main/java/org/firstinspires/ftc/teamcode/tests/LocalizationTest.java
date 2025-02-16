package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.util.Drawing;

@TeleOp(name = "Localization Test", group = "Tests")
public class LocalizationTest extends CommandOpMode {
    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DriveSubsystem chassis = new DriveSubsystem(hardwareMap);

        GamepadEx driver1 = new GamepadEx(gamepad1);

        chassis.setMotorBehavior(Motor.ZeroPowerBehavior.BRAKE);

        chassis.setTeleOpMode(true, true);
        chassis.setTeleOpAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        chassis.setPosition(0, 0, 0);

        schedule(
                new RunCommand(() -> {
                    Pose2D currentPosition = chassis.getPosition();
                    telemetry.addData("Current Position",
                            "X: %.2f in, Y: %.2f in, H: %.2f deg",
                            currentPosition.getX(DistanceUnit.INCH),
                            currentPosition.getY(DistanceUnit.INCH),
                            currentPosition.getHeading(AngleUnit.DEGREES)
                    );

                    Drawing.drawRobot(currentPosition, "#4CAF50");
                    Drawing.sendPacket();

                    telemetry.update();
                })
        );
    }
}
