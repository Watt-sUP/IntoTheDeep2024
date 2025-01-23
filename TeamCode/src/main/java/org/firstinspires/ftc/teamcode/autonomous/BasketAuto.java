package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.assets.AutonomousOpMode;
import org.firstinspires.ftc.teamcode.commands.FollowPointCommand;
import org.firstinspires.ftc.teamcode.util.FixedSequentialCommandGroup;

@Disabled
@Autonomous(name = "Basket Autonomous", group = "Auto Experiments")
public class BasketAuto extends AutonomousOpMode {
    public static final Pose BASKET_POSE = new Pose(18, 122, Math.toRadians(315));

    @Override
    public void initialize() {
        super.initialize();
        startBasket();
        enableInit();

        schedule(
                new FixedSequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        
                        new FollowPointCommand(follower, BASKET_POSE, 1)
                )
        );
    }
}
