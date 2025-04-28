package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
@TeleOp(name = "transmission", group = "drive")
public class transmission extends OpMode {

    private List<LynxModule> allHubs;
    private ElapsedTime elapsedtime;

    private DcMotorEx frMotor;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        frMotor = hardwareMap.get(DcMotorEx.class, "fr_motor");

        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
    }

    @Override
    public void loop() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        double power = Math.pow(-gamepad1.right_stick_y,3);

        if(gamepad1.right_stick_y > .2 || gamepad1.right_stick_y<-.2){
            frMotor.setPower(power);
        }
        else {
            frMotor.setPower(0);
        }

        telemetry.addData("Run time", getRuntime());
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();

        telemetry.addData("y", gamepad1.right_stick_y);
        telemetry.addData("power", power);
        telemetry.update();
    }
}