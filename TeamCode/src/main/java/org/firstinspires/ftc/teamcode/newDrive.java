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

import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import com.qualcomm.hardware.lynx.LynxModule;

@Config
@TeleOp(name = "Drive", group = "drive")
public class newDrive extends OpMode {
    public LynxModule CONTROL_HUB, EXPANSION_HUB;


    private ElapsedTime elapsedtime;
    private List<LynxModule> allHubs;

    DcMotorEx frMotor, flMotor, brMotor, blMotor;
    CRServo frServo, flServo, brServo, blServo;
    AnalogInput frEncoder, flEncoder, brEncoder, blEncoder;
    double frTargetAngle, flTargetAngle, brTargetAngle, blTargetAngle;
    double frServoPower, flServoPower, brServoPower, blServoPower;
    double frRawServoPos, flRawServoPos, brRawServoPos, blRawServoPos;
    double frServoPos, flServoPos, brServoPos, blServoPos;

    double power;
    double rawAngle;
    double angle;

    public static double frOffset = 114.387, flOffset = 202.15, brOffset = 142.99, blOffset = 181.45; // fr 19.72, fl , br, bl


    // PID Controllers
    PIDController frController, flController, brController, blController;
    public static double flP = 0.015, flI = 0, flD = 0;
    public static double frP = .01, frI = 0, frD = 0;
    public static double blP = .01, blI = 0, blD = 0;
    public static double brP = .012, brI = 0, brD = 0;
    BNO055IMU imu;

    @Override
    public void init() {

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frMotor = hardwareMap.get(DcMotorEx.class, "fr_motor");
        flMotor = hardwareMap.get(DcMotorEx.class, "fl_motor");
        brMotor = hardwareMap.get(DcMotorEx.class, "br_motor");
        blMotor = hardwareMap.get(DcMotorEx.class, "bl_motor");
        frMotor.setDirection(DcMotorEx.Direction.REVERSE);
        brMotor.setDirection(DcMotorEx.Direction.REVERSE);
        blMotor.setDirection(DcMotorEx.Direction.REVERSE);


        frServo = hardwareMap.get(CRServo.class, "fr_servo");
        flServo = hardwareMap.get(CRServo.class, "fl_servo");
        brServo = hardwareMap.get(CRServo.class, "br_servo");
        blServo = hardwareMap.get(CRServo.class, "bl_servo");

        frEncoder = hardwareMap.get(AnalogInput.class, "fr_encoder");
        flEncoder = hardwareMap.get(AnalogInput.class, "fl_encoder");
        brEncoder = hardwareMap.get(AnalogInput.class, "br_encoder");
        blEncoder = hardwareMap.get(AnalogInput.class, "bl_encoder");

        frController = new PIDController(frP, frI, frD);
        flController = new PIDController(flP, flI, flD);
        brController = new PIDController(brP, brI, brD);
        blController = new PIDController(blP, blI, blD);

        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
    }

    @Override
    public void loop() {

        for (LynxModule hub : allHubs){
            hub.clearBulkCache();
        }


        frRawServoPos = (frEncoder.getVoltage() / 3.3) * 360;
        frServoPos = frRawServoPos - frOffset;

        flRawServoPos = (flEncoder.getVoltage() / 3.3) * 360;
        flServoPos = flRawServoPos - flOffset;

        brRawServoPos = (brEncoder.getVoltage() / 3.3) * 360;
        brServoPos = brRawServoPos - brOffset;

        blRawServoPos = (blEncoder.getVoltage() / 3.3) * 360;
        blServoPos = blRawServoPos - blOffset;

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        if (y==0){
            y=-y;
        }
        rawAngle = Math.toDegrees(Math.atan2(y,x));
        angle = rawAngle - 90;

        if(x>.2 || y>.2 || x< -.2 || y < -.2) {
            power = Math.pow(Math.sqrt(x * x + y * y), 3);
        }
        else {
            power = 0;
            angle = 0;
        }
        if (angle <=90 && angle >= -90) {
            frTargetAngle = angle;
            flTargetAngle = angle;
            brTargetAngle = angle;
            blTargetAngle = angle;
        }
        if(angle < -90 && angle >= -270) {
            frTargetAngle = angle + 180;
            flTargetAngle = angle + 180;
            brTargetAngle = angle + 180;
            blTargetAngle = angle + 180;
            power = -power;
        }

        frMotor.setPower(power);
        flMotor.setPower(power);
        brMotor.setPower(power);
        blMotor.setPower(power);

        frServoPower = frController.calculate(frServoPos, frTargetAngle);
        flServoPower = flController.calculate(flServoPos, flTargetAngle);
        brServoPower = brController.calculate(brServoPos, brTargetAngle);
        blServoPower = blController.calculate(blServoPos, blTargetAngle);

        frServo.setPower(frServoPower);
        flServo.setPower(flServoPower);
        brServo.setPower(brServoPower);
        blServo.setPower(blServoPower);

        telemetry.addData("Run time", getRuntime());
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();

        telemetry.addData("raw angle", rawAngle);
        telemetry.addData("Angle", angle);
        telemetry.addData("power", power);
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.update();
    }
}