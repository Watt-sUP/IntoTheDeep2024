package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.LimelightRelocalization;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

@Config
@TeleOp
public class LimelightTest extends CommandOpMode {
    public static double startingAngle = 0;

    @Override
    public void initialize() {
        Constants.setConstants(FConstants.class, LConstants.class);
        LimelightSubsystem LLsystem = new LimelightSubsystem(hardwareMap);
        Follower follower = new Follower(hardwareMap);

        follower.setStartingPose(new Pose(7.95, 66, Math.toRadians(180)));
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        telemetry.setMsTransmissionInterval(10);
        LLsystem.setPipeline(0);

//        SparkFunOTOS otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
//
//        otos.setOffset(new SparkFunOTOS.Pose2D(-6.8897637795275, 0, 0));
//
//        otos.setLinearScalar(.9665);
//        otos.setAngularUnit(AngleUnit.DEGREES);
//        otos.setAngularScalar(.994);
//
//        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, startingAngle));
//        otos.calibrateImu();
//        otos.resetTracking();

        GamepadEx driver1 = new GamepadEx(gamepad1);

        DriveSubsystem chassis = new DriveSubsystem(hardwareMap);
        chassis.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);

        register(chassis, intake);
        schedule(
//                new RunCommand(() -> {
//                    Pose botPose = LLsystem.getBotPose(otos.getPosition().h);
//                    if (botPose != null) {
//                        telemetry.addData("Bot Pose", botPose);
//                        telemetry.addData("Bot Pose (Pedro)", LimelightSubsystem.toPedroPose(botPose));
//                        telemetry.addData("Bot Pose (Pedro, Neutral)", LimelightSubsystem.toPedroPoseNeutral(botPose));
//
//                        Drawing.drawRobot(botPose, "#4CAF50");
//                        Drawing.sendPacket();
//                    } else telemetry.addLine("No AprilTag in sight!");
//                    telemetry.update();
//                }),
                new LimelightRelocalization(LLsystem, follower)
                        .enableUpdates(false),
                new RunCommand(follower::update)
        );
    }
}
