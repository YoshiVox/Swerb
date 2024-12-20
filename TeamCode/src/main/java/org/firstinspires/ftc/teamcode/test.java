package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "test")
public class test extends OpMode {

    AnalogInput encoder;
    ServoImplEx servo;
    public static double servopos =.5;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        encoder = hardwareMap.get(AnalogInput.class, "eFL");
        servo = hardwareMap.get(ServoImplEx.class, "sFL");
        servo.setPwmRange(new PwmControl.PwmRange(505, 2495));

    }

    @Override
    public void loop() {

        double position = encoder.getVoltage() / 3.3 * 360;

        // Telemetry
        telemetry.addData("1", "test");
        telemetry.addData("Encoder Position", position);
        telemetry.addData("Servo Position", servopos);
        telemetry.update();

        if (gamepad1.a) {
            servo.setPosition(servopos); // Example test position
        }
    }
}
