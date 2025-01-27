package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
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

@TeleOp
public class LimelightTest extends CommandOpMode {

    @Override
    public void initialize() {
        Constants.setConstants(FConstants.class, LConstants.class);
        LimelightSubsystem LLsystem = new LimelightSubsystem(hardwareMap);
        Follower follower = new Follower(hardwareMap);

        follower.setStartingPose(new Pose(7.95, 66, Math.toRadians(180)));
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        telemetry.setMsTransmissionInterval(10);

        GamepadEx driver1 = new GamepadEx(gamepad1);
        DriveSubsystem chassis = new DriveSubsystem(hardwareMap);
        chassis.setAxes(driver1::getLeftY, driver1::getLeftX, driver1::getRightX);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        intake.setExtendoState(IntakeSubsystem.ExtendoState.IN);

        register(chassis, intake);
        schedule(
                new LimelightRelocalization(LLsystem, follower)
                        .enableUpdates(false),
                new RunCommand(follower::update)
        );
    }
}
