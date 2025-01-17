package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp
@Config
public class IMUTest extends LinearOpMode {
    public static String IMU_NAME = "imu_stable";
    public static double ANGULAR_GAIN = 0.992, LINEAR_GAIN = 1;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        IMU imu = hardwareMap.get(IMU.class, "imu_stable");
        SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));
        imu.resetYaw();

        telemetry.addLine("Calibrating OTOS...");
        telemetry.update();

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        otos.setAngularScalar(ANGULAR_GAIN);
        otos.setLinearScalar(LINEAR_GAIN);
        otos.setOffset(new SparkFunOTOS.Pose2D(-6.8897637795275, 0, 0));

        otos.calibrateImu(255, true);
        otos.resetTracking();

        telemetry.addLine("OTOS Calibrated!");
        telemetry.update();

        waitForStart();

        GamepadEx driver1 = new GamepadEx(gamepad1);

        DriveSubsystem chassis = new DriveSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);

        chassis.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        while (opModeIsActive()) {
            if (gamepad1.y) {
                otos.resetTracking();
            }
            chassis.periodic();

            telemetry.addData("IMU Angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("OTOS Pose", otos.getPosition().toString());
            telemetry.addData("OTOS Angular Gain", otos.getAngularScalar());
            telemetry.update();
        }
    }
}
