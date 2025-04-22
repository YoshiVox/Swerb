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
@TeleOp(name = "swerve_pid_fieldcentric", group = "drive")
public class drive extends OpMode {

    DcMotorEx frontRight, frontLeft, backRight, backLeft;
    CRServo frServo, flServo, brServo, blServo;
    AnalogInput frEncoder, flEncoder, brEncoder, blEncoder;

    public static double frOffset = -15.5, flOffset = 52.5, brOffset = -7.5, blOffset = 10;

    // PID Controllers
    PIDController frPID = new PIDController(0.015, 0, 0);
    PIDController flPID = new PIDController(0.025, 0, 0);
    PIDController brPID = new PIDController(0.012, 0, 0);
    PIDController blPID = new PIDController(0.015, 0, 0);

    BNO055IMU imu;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frontRight = hardwareMap.get(DcMotorEx.class, "fr_motor");
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl_motor");
        backRight = hardwareMap.get(DcMotorEx.class, "br_motor");
        backLeft = hardwareMap.get(DcMotorEx.class, "bl_motor");

        frServo = hardwareMap.get(CRServo.class, "fr_servo");
        flServo = hardwareMap.get(CRServo.class, "fl_servo");
        brServo = hardwareMap.get(CRServo.class, "br_servo");
        blServo = hardwareMap.get(CRServo.class, "bl_servo");

        frEncoder = hardwareMap.get(AnalogInput.class, "fr_encoder");
        flEncoder = hardwareMap.get(AnalogInput.class, "fl_encoder");
        brEncoder = hardwareMap.get(AnalogInput.class, "br_encoder");
        blEncoder = hardwareMap.get(AnalogInput.class, "bl_encoder");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    @Override
    public void loop() {
        double leftX = gamepad1.left_stick_x;
        double leftY = -gamepad1.left_stick_y;
        double rightX = gamepad1.right_stick_x;

        double botHeading = imu.getAngularOrientation().firstAngle;

        // Field-centric transformation
        double rotX = leftX * Math.cos(Math.toRadians(botHeading)) - leftY * Math.sin(Math.toRadians(botHeading));
        double rotY = leftX * Math.sin(Math.toRadians(botHeading)) + leftY * Math.cos(Math.toRadians(botHeading));

        // Compute vector angle and power
        double magnitude = Math.hypot(rotX, rotY);
        double moveAngle = Math.toDegrees(Math.atan2(rotY, rotX));
        moveAngle = (moveAngle + 360) % 360;

        // Apply desired rotation to all modules
        applySwerveModule(frontRight, frServo, frEncoder, frPID, frOffset, moveAngle, magnitude, rightX);
        applySwerveModule(frontLeft, flServo, flEncoder, flPID, flOffset, moveAngle, magnitude, -rightX);
        applySwerveModule(backRight, brServo, brEncoder, brPID, brOffset, moveAngle, magnitude, rightX);
        applySwerveModule(backLeft, blServo, blEncoder, blPID, blOffset, moveAngle, magnitude, -rightX);

        telemetry.addData("Heading", botHeading);
        telemetry.addData("Target Angle", moveAngle);
        telemetry.update();
    }

    void applySwerveModule(DcMotorEx driveMotor, CRServo turningServo, AnalogInput encoder, PIDController pid,
                           double offset, double moveAngle, double magnitude, double rotationComp) {

        double rawAngle = encoder.getVoltage() / 3.3 * 360;
        double currentAngle = (rawAngle - offset + 360) % 360;

        // Get shortest angle difference
        double angleError = angleWrap(moveAngle - currentAngle);
        boolean reverseDrive = false;

        // Flip direction if > 90° to reduce rotation
        if (Math.abs(angleError) > 90) {
            angleError = angleWrap(angleError + 180);
            reverseDrive = true;
        }

        // PID output to control CRServo
        double pidOutput = pid.calculate(currentAngle, moveAngle);
        pidOutput = Math.max(Math.min(pidOutput, 1), -1);
        turningServo.setPower(pidOutput);

        // Set motor power with reverse logic
        double drivePower = magnitude;
        if (reverseDrive) drivePower *= -1;
        driveMotor.setPower(drivePower + rotationComp);
    }

    // Wrap angle to [-180, 180]
    double angleWrap(double angle) {
        angle = (angle + 180) % 360;
        if (angle < 0) angle += 360;
        return angle - 180;
    }
}
