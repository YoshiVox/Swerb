package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp(name = "Pod Test")
public class podTest extends OpMode {

    AnalogInput efl;

    DcMotorEx mfl;
    ServoImplEx fl;

    public static double FLservopos = .5;



    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        mfl =hardwareMap.get(DcMotorEx.class, "mfl");
        efl = hardwareMap.get(AnalogInput.class, "eFL");
        fl = hardwareMap.get(ServoImplEx.class, "sFL");
        fl.setPwmRange(new PwmControl.PwmRange(505, 2495));

    }

    @Override
    public void loop() {

        double FLposition = efl.getVoltage() / 3.3 * 360;


        // Telemetry
        telemetry.addData("1", "test");
        telemetry.addData("FL Position", FLposition);

        telemetry.update();

        if (gamepad1.right_stick_y > .2 || gamepad1.right_stick_y < -.2){
            mfl.setPower(gamepad1.right_stick_y);
        }
        else {
            mfl.setPower(0);
        }

        if (gamepad1.left_stick_x > .1) {
            fl.setPosition(gamepad1.left_stick_x);
        }

    }
}
