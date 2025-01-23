package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@Config
@TeleOp(name = "Intake Pivot Tuner", group = "Tuners")
public class IntakePivotTuner extends LinearOpMode {
    public static double POSITION_LEFT = 0;
    public static double POSITION_RIGHT = 0;

    @Override
    public void runOpMode() {
        SimpleServo pivLeft = new SimpleServo(hardwareMap, "int_left", 0, 360);
        SimpleServo pivRight = new SimpleServo(hardwareMap, "int_right", 0, 360);

        pivLeft.setInverted(true);
        pivRight.setInverted(false);

        waitForStart();

        while (opModeIsActive()) {
            pivLeft.turnToAngle(POSITION_LEFT);
            pivRight.turnToAngle(POSITION_RIGHT);
        }
    }
}
