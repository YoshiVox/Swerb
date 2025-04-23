package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "pod", group = "drive")
public class pod extends OpMode {

    DcMotorEx frMotor;
    CRServo frServo;
    AnalogInput frEncoder;
    double targetAngle;
    double servoPower;
    double servoPos;

    double power;

    double angle;

    public static double frOffset = 73.25;

    // PID Controllers
    PIDController controller;
    public static final double P = 0.005, I = 0, D = 0;

    BNO055IMU imu;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frMotor = hardwareMap.get(DcMotorEx.class, "fr_motor");

        frServo = hardwareMap.get(CRServo.class, "fr_servo");

        frEncoder = hardwareMap.get(AnalogInput.class, "fr_encoder");

        controller = new PIDController(P, I, D);

    }

    @Override
    public void loop() {
        servoPos = (frEncoder.getVoltage() / 3.3) * 360 - frOffset;

/*
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;


        // Compute vector angle and power
        if(x>.2 || y>.2) {
            power = Math.pow(Math.sqrt(x * x + y * y), 3);
        }
        else {
            power = 0;
        }
        angle = Math.toDegrees(Math.atan2(y,x))-90;

        if (angle <=90 && angle >= -90) {
            targetAngle = angle;
        }
        if(angle > 90) {
            targetAngle = angle - 180;
            power = -power;
        }
*/
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y; // Negative because up is usually -Y

// Calculate angle (in degrees), rotated so 0° is "up"
        double rawAngle = Math.toDegrees(Math.atan2(y, x));
        double angle = rawAngle - 90;

// Normalize angle to [0, 360)
        angle = (angle + 360) % 360;

// Set power (with cubic scaling)
        double power = Math.sqrt(x * x + y * y);
        power = (power > 0.2) ? Math.pow(power, 3) : 0;

// Transform angle according to your rule
        double targetAngle;

        if (angle >= 270 || angle <= 90) {
            // From -90° to +90°, use angle as-is (map 270 → -90, 0 → -90, 90 → 0)
            if (angle >= 270) {
                targetAngle = angle - 360; // ex: 315 → -45
            } else {
                targetAngle = angle; // ex: 45 → 45
            }
        } else {
            // From 90° to 270°, mirror angle around 180° and reverse power
            targetAngle = 180 - angle;
            power = -power;
        }
        frMotor.setPower(power);

        servoPower = controller.calculate(servoPos, targetAngle);
        frServo.setPower(servoPower);


        telemetry.addData("Angle", angle);
        telemetry.addData("Target Angle", targetAngle);
        telemetry.addData("Servo Position", servoPos);
        telemetry.addData("Servo Power", servoPower);
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Power", power);
        telemetry.update();
    }
}