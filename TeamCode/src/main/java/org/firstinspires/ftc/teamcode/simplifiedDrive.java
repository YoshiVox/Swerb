package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
@TeleOp(name = "simplified Drive", group = "drive")
public class simplifiedDrive extends OpMode {

    private final ElapsedTime elapsedtime = new ElapsedTime();
    private List<LynxModule> allHubs;

    DcMotorEx[] motors = new DcMotorEx[4];
    CRServo[] servos = new CRServo[4];
    AnalogInput[] encoders = new AnalogInput[4];
    PIDController[] controllers = new PIDController[4];

    double[] servoOffsets = {114.387, 0, 0, 0}; // fr, fl, br, bl
    double[] servoPositions = new double[4];
    double[] targetAngles = new double[4];
    double[] servoPowers = new double[4];

    public static double[] P = {.01, 0.025, .012, .01}; // fr, fl, br, bl
    public static double[] I = {0, 0, 0, 0};
    public static double[] D = {0, 0, 0, 0};

    double angle = 0, power = 0;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        String[] motorNames = {"fr_motor", "fl_motor", "br_motor", "bl_motor"};
        String[] servoNames = {"fr_servo", "fl_servo", "br_servo", "bl_servo"};
        String[] encoderNames = {"fr_encoder", "fl_encoder", "br_encoder", "bl_encoder"};

        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, motorNames[i]);
            servos[i] = hardwareMap.get(CRServo.class, servoNames[i]);
            encoders[i] = hardwareMap.get(AnalogInput.class, encoderNames[i]);
            controllers[i] = new PIDController(P[i], I[i], D[i]);
        }

        elapsedtime.reset();
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) hub.clearBulkCache();

        // Calculate servo positions
        for (int i = 0; i < 4; i++) {
            double rawPos = (encoders[i].getVoltage() / 3.3) * 360;
            servoPositions[i] = rawPos - servoOffsets[i];
        }

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;

        angle = Math.toDegrees(Math.atan2(y, x)) - 90;

        if (Math.abs(x) > 0.2 || Math.abs(y) > 0.2) {
            power = Math.pow(Math.hypot(x, y), 3);
        } else {
            power = 0;
            angle = 0;
        }

        // Adjust angle and power if in reverse quadrant
        if (angle < -90 && angle >= -270) {
            angle += 180;
            power = -power;
        }

        // Apply same target angle to all
        for (int i = 0; i < 4; i++) {
            targetAngles[i] = angle;
            motors[i].setPower(power);
            servoPowers[i] = controllers[i].calculate(servoPositions[i], targetAngles[i]);
            servos[i].setPower(servoPowers[i]);
        }

        telemetry.addData("Run time", getRuntime());
        telemetry.addData("Loop Time (ms)", elapsedtime.milliseconds());
        telemetry.addData("Angle", angle);
        telemetry.addData("Power", power);
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.update();

        elapsedtime.reset();
    }
}
