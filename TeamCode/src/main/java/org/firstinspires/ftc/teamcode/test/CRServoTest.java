package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
@Config
@TeleOp(name = "CRServo", group = "test")
public class CRServoTest extends OpMode {

    private PIDController frPID;
    private PIDController flPID;
    private PIDController brPID;
    private PIDController blPID;


    public static double fr = 116;
    public static double fl = 205;
    public static double bl = 184;
    public static double br = 145;
    public static double flP = 0.01, flI = 0, flD = 0;
    public static double frP = .01, frI = 0, frD = 0;
    public static double blP = .015, blI = 0, blD = 0;
    public static double brP = .012, brI = 0, brD = 0;


    // CRServos
    CRServo frServo, flServo, brServo, blServo;

    // Analog Encoders
    AnalogInput frEncoder, flEncoder, brEncoder, blEncoder;

    // Offsets (can be saved later)
    double frOffset = 116;
    double flOffset = 205;
    double brOffset = 145;
    double blOffset = 184;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frPID = new PIDController(frP, frI, frD);
        flPID = new PIDController(flP, flI, flD);
        brPID = new PIDController(brP, brI, brD);
        blPID = new PIDController(blP, blI, blD);

        // Servos
        frServo = hardwareMap.get(CRServo.class, "fr_servo");
        flServo = hardwareMap.get(CRServo.class, "fl_servo");
        brServo = hardwareMap.get(CRServo.class, "br_servo");
        blServo = hardwareMap.get(CRServo.class, "bl_servo");

        // Encoders
        frEncoder = hardwareMap.get(AnalogInput.class, "fr_encoder");
        flEncoder = hardwareMap.get(AnalogInput.class, "fl_encoder");
        brEncoder = hardwareMap.get(AnalogInput.class, "br_encoder");
        blEncoder = hardwareMap.get(AnalogInput.class, "bl_encoder");
    }

    @Override
    public void loop() {
        double frPos = (frEncoder.getVoltage() / 3.3) * 360;
        double flPos = (flEncoder.getVoltage() / 3.3) * 360;
        double brPos = (brEncoder.getVoltage() / 3.3) * 360;
        double blPos = (blEncoder.getVoltage() / 3.3) * 360;

        // Set offsets with buttons
        if (gamepad1.a){ frOffset = fr;}
        if (gamepad1.b) {flOffset = fl;}
        if (gamepad1.x) {brOffset = br;}
        if (gamepad1.y) {blOffset = bl;}

        // PID to move servos to offsets
        double frPower = frPID.calculate(frPos, frOffset);
        double flPower = flPID.calculate(flPos, flOffset);
        double brPower = brPID.calculate(brPos, brOffset);
        double blPower = blPID.calculate(blPos, blOffset);

        frServo.setPower(frPower);
        flServo.setPower(flPower);
        brServo.setPower(brPower);
        blServo.setPower(blPower);

        // Telemetry
        telemetry.addLine("Press A/B/X/Y to set zero offset for:");
        telemetry.addLine("  A = Front Right");
        telemetry.addLine("  B = Front Left");
        telemetry.addLine("  X = Back Right");
        telemetry.addLine("  Y = Back Left");
        telemetry.addLine();

        telemetry.addData("FR Pos", "%.2f deg", frPos);
        telemetry.addData("FL Pos", "%.2f deg", flPos);
        telemetry.addData("BR Pos", "%.2f deg", brPos);
        telemetry.addData("BL Pos", "%.2f deg", blPos);

        telemetry.addData("FR Offset", "%.2f", frOffset);
        telemetry.addData("FL Offset", "%.2f", flOffset);
        telemetry.addData("BR Offset", "%.2f", brOffset);
        telemetry.addData("BL Offset", "%.2f", blOffset);

        telemetry.addData("FR Power", "%.2f", frPower);
        telemetry.addData("FL Power", "%.2f", flPower);
        telemetry.addData("BR Power", "%.2f", brPower);
        telemetry.addData("BL Power", "%.2f", blPower);

        telemetry.update();
    }
}
