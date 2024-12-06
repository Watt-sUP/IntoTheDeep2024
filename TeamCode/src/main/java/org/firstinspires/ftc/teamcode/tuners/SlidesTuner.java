package org.firstinspires.ftc.teamcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "Slides Tuner", group = "Tuners")
public class SlidesTuner extends CommandOpMode {
    public static int POSITION = 0;
//    public static double kP = 0.01, kI = 0.55, kD = 0.0009, kF = 0.00009;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor slidesMotor1 = hardwareMap.dcMotor.get("slides");
        slidesMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor slidesMotor2 = hardwareMap.dcMotor.get("slides2");
        slidesMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

        slidesMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidesMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slidesMotor1.setTargetPosition(POSITION);
        slidesMotor2.setTargetPosition(POSITION);

        slidesMotor1.setPower(1);
        slidesMotor2.setPower(1);

        slidesMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidesMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        schedule(
                new RunCommand(() -> {
                    slidesMotor1.setTargetPosition(POSITION);
                    slidesMotor2.setTargetPosition(POSITION);

                    telemetry.addData("pos1", slidesMotor1.getCurrentPosition());
                    telemetry.addData("pos2", slidesMotor2.getCurrentPosition());
                    telemetry.update();
                })
        );
    }
}
