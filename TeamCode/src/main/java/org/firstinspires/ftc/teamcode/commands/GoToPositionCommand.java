package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class GoToPositionCommand extends CommandBase {
    private final DriveSubsystem drive;
    private final Pose2D pose;

    public GoToPositionCommand(DriveSubsystem drive, Pose2D pose) {
        this.drive = drive;
        this.pose = pose;

        addRequirements(drive);

        drive.setMaxSpeed(1);
        drive.goToPosition(pose);
    }

    public GoToPositionCommand(DriveSubsystem drive, double x, double y, double h) {
        this(drive, new Pose2D(
                DistanceUnit.INCH,
                x,
                y,
                AngleUnit.DEGREES,
                AngleUnit.normalizeDegrees(h)
        ));
    }

    public GoToPositionCommand setMaxSpeed(double speed) {
        drive.setMaxSpeed(speed);
        return this;
    }

    @Override
    public void execute() {
        drive.goToPosition(pose);
    }

    @Override
    public boolean isFinished() {
        return drive.atTarget();
    }
}
