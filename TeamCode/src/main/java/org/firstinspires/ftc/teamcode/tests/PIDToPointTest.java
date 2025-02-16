package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;

@Config
@Autonomous
public class PIDToPointTest extends LinearOpMode {
    public static double hP = -3, hI = 0, hD = -18;
    public static double fP = 0.12, fD = 0.008;
    public static double sP = -0.5, sD = -0.005;
    public GoBildaPinpointDriver localization;
    public DcMotorEx fr, fl, br, bl;
    double headingIntegral = 0;
    double lastHeadingError = 0;
    double lastForwardError = 0;
    double lastStrafeError = 0;

    public double pidHeading(double target, double current) {
        double error = target - current;
        headingIntegral += error;
        double derivative = error - lastHeadingError;
        if (error > Math.PI) {
            error -= 2 * Math.PI;
        } else if (error < -Math.PI) {
            error += 2 * Math.PI;
        }
        double correction = (error * hP) + (headingIntegral * hI) + (derivative * hD);
        lastHeadingError = error;
        return correction;
    }

    public double pdForward(double error) {
        double derivative = error - lastForwardError;
        double correction = (error * fP) + (derivative * fD);
        lastForwardError = error;
        return correction;
    }

    public double pdStrafe(double error) {
        double derivative = error - lastStrafeError;
        double correction = (error * sP) + (derivative * sD);
        lastStrafeError = error;
        return correction;
    }

    public void goToPosition(double x, double y, double h, double speed) {
        localization.update();

        Vector2d driveVector = new Vector2d(x - localization.getPosition().getX(DistanceUnit.INCH), y - localization.getPosition().getY(DistanceUnit.INCH));
        Vector2d rotatedVector = driveVector.rotateBy(AngleUnit.RADIANS.toDegrees(localization.getHeading()));

        double inputTurn = pidHeading(Math.toRadians(h), localization.getHeading());
        double driveCorrection = pdForward(rotatedVector.getX());
        double strafeCorrection = pdStrafe(rotatedVector.getY());

        fr.setPower((driveCorrection - strafeCorrection - inputTurn) * speed);
        fl.setPower((driveCorrection + strafeCorrection + inputTurn) * speed);
        br.setPower((driveCorrection + strafeCorrection - inputTurn) * speed);
        bl.setPower((driveCorrection - strafeCorrection + inputTurn) * speed);
    }

    @Override
    public void runOpMode() {
        fr = hardwareMap.get(DcMotorEx.class, "rightFront");
        fl = hardwareMap.get(DcMotorEx.class, "leftFront");
        br = hardwareMap.get(DcMotorEx.class, "rightBack");
        bl = hardwareMap.get(DcMotorEx.class, "leftBack");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        localization = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        localization.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        localization.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );
        localization.setOffsets(-117.5, 65);

        localization.resetPosAndIMU();

        ElapsedTime timer = new ElapsedTime();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            goToPosition(0, 0, 0, 1);
        }
    }
}
