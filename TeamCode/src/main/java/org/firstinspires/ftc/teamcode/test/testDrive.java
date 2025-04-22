package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "testDrive", group= "test")
public class testDrive extends OpMode {

    // Motors
    DcMotorEx frontRight;
    DcMotorEx frontLeft;
    DcMotorEx backRight;
    DcMotorEx backLeft;

    // Servos
    ServoImplEx frontRightServo;
    ServoImplEx frontLeftServo;
    ServoImplEx backRightServo;
    ServoImplEx backLeftServo;

    // Servo Encoders
    AnalogInput frontRightEncoder;
    AnalogInput frontLeftEncoder;
    AnalogInput backRightEncoder;
    AnalogInput backLeftEncoder;

    private boolean frontRightReversed = false;
    private boolean frontLeftReversed = true; // Example: reversed
    private boolean backRightReversed = true;
    private boolean backLeftReversed = false; // Example: reversed
    // Zero Positions
    private double frontRightZero = 269.2; //.25
    private double frontLeftZero = 247.8; //.68
    private double backRightZero = 170.29; //.47
    private double backLeftZero = 90.65;

    // Constants
    private final double MAX_SERVO_ANGLE = 360.0;
    private final double SERVO_RANGE = 1.0; // Servo range in code (0 to 1)

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize motors
        frontRight = hardwareMap.get(DcMotorEx.class, "fr_motor");
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl_motor");
        backRight = hardwareMap.get(DcMotorEx.class, "br_motor");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl_motor");

        // Initialize servos
        frontRightServo = hardwareMap.get(ServoImplEx.class, "fr_servo");
        frontRightServo.setPwmRange(new PwmControl.PwmRange(505, 2495));
        frontLeftServo = hardwareMap.get(ServoImplEx.class, "fl_servo");
        frontLeftServo.setPwmRange(new PwmControl.PwmRange(505, 2495));
        backRightServo = hardwareMap.get(ServoImplEx.class, "br_servo");
        backRightServo.setPwmRange(new PwmControl.PwmRange(505, 2495));
        backLeftServo = hardwareMap.get(ServoImplEx.class, "bl_servo");
        backLeftServo.setPwmRange(new PwmControl.PwmRange(505, 2495));

        // Initialize encoders
        frontRightEncoder = hardwareMap.get(AnalogInput.class, "fr_encoder");
        frontLeftEncoder = hardwareMap.get(AnalogInput.class, "fl_encoder");
        backRightEncoder = hardwareMap.get(AnalogInput.class, "br_encoder");
        backLeftEncoder = hardwareMap.get(AnalogInput.class, "bl_encoder");

        // Set initial servo positions
        frontRightServo.setPosition(calculateServoPosition(frontRightEncoder.getVoltage() / 3.3 * 360, frontRightZero, frontRightReversed));
        frontLeftServo.setPosition(calculateServoPosition(frontLeftEncoder.getVoltage() / 3.3 * 360, frontLeftZero, frontLeftReversed));
        backRightServo.setPosition(calculateServoPosition(backRightEncoder.getVoltage() / 3.3 * 360, backRightZero, backRightReversed));
        backLeftServo.setPosition(calculateServoPosition(backLeftEncoder.getVoltage() / 3.3 * 360, backLeftZero, backLeftReversed));

        double frpos = frontRightEncoder.getVoltage() / 3.3 * 360;
        double flpos = frontLeftEncoder.getVoltage() / 3.3 * 360;
        double brpos = backRightEncoder.getVoltage() / 3.3 * 360;
        double blpos = backLeftEncoder.getVoltage() / 3.3 * 360;

        telemetry.addData("FR pos", frpos);
        telemetry.addData("FL pos", flpos);
        telemetry.addData("BR pos", brpos);
        telemetry.addData("BL pos", blpos);
    }

    @Override
    public void loop() {
        // Read joystick inputs with dead zone
        double x = Math.abs(gamepad1.left_stick_x) > 0.05 ? gamepad1.left_stick_x : 0;
        double y = Math.abs(gamepad1.left_stick_y) > 0.05 ? -gamepad1.left_stick_y : 0;
        double rotation = Math.abs(gamepad1.right_stick_x) > 0.05 ? gamepad1.right_stick_x : 0;

        // Calculate vector magnitudes and angles
        double magnitude = Math.hypot(x, y);
        double angle = Math.atan2(y, x) - Math.PI / 4;

        // Calculate individual wheel speeds
        double frontRightPower = magnitude * Math.cos(angle) + rotation;
        double frontLeftPower = magnitude * Math.sin(angle) - rotation;
        double backRightPower = magnitude * Math.sin(angle) + rotation;
        double backLeftPower = magnitude * Math.cos(angle) - rotation;

        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(1.0, Math.max(Math.abs(frontRightPower),
                Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backRightPower), Math.abs(backLeftPower)))));
        frontRightPower /= maxPower;
        frontLeftPower /= maxPower;
        backRightPower /= maxPower;
        backLeftPower /= maxPower;

        // Set motor powers
        frontRight.setPower(frontRightPower);
        frontLeft.setPower(frontLeftPower);
        backRight.setPower(backRightPower);
        backLeft.setPower(backLeftPower);

        // Stabilize servo positions
        double frontRightServoPos = calculateServoPosition(frontRightEncoder.getVoltage() / 3.3 * 360, frontRightZero, frontRightReversed);
        double frontLeftServoPos = calculateServoPosition(frontLeftEncoder.getVoltage() / 3.3 * 360, frontLeftZero, frontLeftReversed);
        double backRightServoPos = calculateServoPosition(backRightEncoder.getVoltage() / 3.3 * 360, backRightZero, backRightReversed);
        double backLeftServoPos = calculateServoPosition(backLeftEncoder.getVoltage() / 3.3 * 360, backLeftZero, backLeftReversed);

        double frpos = frontRightEncoder.getVoltage() / 3.3 * 360;
        double flpos = frontLeftEncoder.getVoltage() / 3.3 * 360;
        double brpos = backRightEncoder.getVoltage() / 3.3 * 360;
        double blpos = backLeftEncoder.getVoltage() / 3.3 * 360;

        frontRightServo.setPosition(frontRightServoPos);
        frontLeftServo.setPosition(frontLeftServoPos);
        backRightServo.setPosition(backRightServoPos);
        backLeftServo.setPosition(backLeftServoPos);

        // Debug telemetry
        telemetry.addData("test", "test");
        telemetry.addData("FR angle", frontRightServoPos);
        telemetry.addData("FL angle", frontLeftServoPos);
        telemetry.addData("BR angle", backRightServoPos);
        telemetry.addData("BL angle", backLeftServoPos);
        telemetry.addData("FR pos", frpos);
        telemetry.addData("FL pos", flpos);
        telemetry.addData("BR pos", brpos);
        telemetry.addData("BL pos", blpos);
        telemetry.update();

    }

    public double calculateServoPosition(double encoderValue, double zeroPosition, boolean reversed) {
        // Calculate adjusted angle
        double adjustedAngle = Math.round((encoderValue - zeroPosition + MAX_SERVO_ANGLE) % MAX_SERVO_ANGLE);
        double servoPosition = (adjustedAngle / MAX_SERVO_ANGLE) * SERVO_RANGE;
        return reversed ? (1.0 - servoPosition) : servoPosition;
    }
}