package robotcode.tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp(name = "Extendo Tuner", group = "Tuners")
public class ExtendoTuner extends LinearOpMode {
    public static double POSITION_LEFT = 0;
    public static double POSITION_RIGHT = 0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SimpleServo extLeft = new SimpleServo(hardwareMap, "ext_left", 0, 220);
        SimpleServo extRight = new SimpleServo(hardwareMap, "ext_right", 0, 220);

        extRight.setInverted(true);

        waitForStart();

        while (opModeIsActive()) {
            extLeft.setPosition(POSITION_LEFT);
            extRight.setPosition(POSITION_RIGHT);
        }
    }
}
