package robotcode.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor Tuner", group = "Tuners")
public class MotorTuner extends LinearOpMode {
    public static String name = "test";
    public static int POSITION = 0;
    public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = (DcMotorEx) hardwareMap.dcMotor.get(name);

        motor.setDirection(DIRECTION);

        motor.setTargetPosition(POSITION);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            motor.setTargetPosition(POSITION);
            motor.setDirection(DIRECTION);
            
            telemetry.addData("Motor Position", motor.getCurrentPosition());
            telemetry.addData("Motor Target", motor.getTargetPosition());
            telemetry.update();
        }
    }
}
