package org.firstinspires.ftc.teamcode.autonomous.experiments;

import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

@Disabled
@Autonomous(name = "Basket Autonomous", group = "Auto Experiments")
public class BasketAuto extends AutonomousOpMode {
    @Override
    public void initialize() {
        super.initialize();
        startBasket();
        enableInit();

        schedule(
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive)
                )
        );
    }
}
