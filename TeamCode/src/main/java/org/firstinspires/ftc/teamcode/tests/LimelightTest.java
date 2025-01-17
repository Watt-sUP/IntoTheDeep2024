package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Drawing;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

@Config
@TeleOp
public class LimelightTest extends CommandOpMode {
    public static AngleUnit angleUnit = AngleUnit.DEGREES;

    @Override
    public void initialize() {
        LimelightSubsystem LLsystem = new LimelightSubsystem(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        telemetry.setMsTransmissionInterval(10);
        LLsystem.setPipeline(0);

        SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setAngularScalar(.992);
        otos.calibrateImu();
        otos.resetTracking();

        GamepadEx driver1 = new GamepadEx(gamepad1);

        DriveSubsystem chassis = new DriveSubsystem(hardwareMap);
        chassis.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);

        register(chassis, intake);
        schedule(new RunCommand(() -> {
            Pose botPose = LLsystem.getBotPose(angleUnit.fromDegrees(otos.getPosition().h));

            telemetry.addData("Bot Pose", botPose);
            telemetry.update();

            if (botPose != null) {
                Drawing.drawRobot(botPose, "#4CAF50");
                Drawing.sendPacket();
            }
        }));
    }
}
