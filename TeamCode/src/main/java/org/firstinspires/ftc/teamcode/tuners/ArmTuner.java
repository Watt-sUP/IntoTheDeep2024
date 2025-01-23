package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Disabled
@Config
@TeleOp(name = "Arm Tuner", group = "Tuners")
public class ArmTuner extends LinearOpMode {
    public static double LEFT_DEGREES = 0;
    public static double RIGHT_DEGREES = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SimpleServo armLeft = new SimpleServo(hardwareMap, "arm_left", 0, 220);
        SimpleServo armRight = new SimpleServo(hardwareMap, "arm_right", 0, 220);

        armLeft.setInverted(true);
        armRight.setInverted(false);

        waitForStart();

        while (opModeIsActive()) {
            armLeft.turnToAngle(LEFT_DEGREES);
            armRight.turnToAngle(RIGHT_DEGREES);

            telemetry.addData("Arm Left Position", armLeft.getAngle(AngleUnit.DEGREES));
            telemetry.addData("Arm Right Position", armRight.getAngle(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
