package org.firstinspires.ftc.teamcode.autonomous.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.commands.GoToPositionCommand;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

@Config
@Autonomous(name = "Forward Tuner", group = "Tuner")
public class ForwardTuner extends AutonomousOpMode {
    public static double DISTANCE = 24;

    @Override
    public void initialize() {
        super.initialize();
        enableInit();

        schedule(
                new RepeatCommand(
                        new FixedSequentialCommandGroup(
                                new GoToPositionCommand(drive, DISTANCE, 0, 0),
                                new GoToPositionCommand(drive, 0, 0, 0)
                        )
                )
        );
    }
}
