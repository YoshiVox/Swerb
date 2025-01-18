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

    AnalogInput efl;
    AnalogInput efr;
    AnalogInput ebl;
    AnalogInput ebr;

    ServoImplEx fl;
    ServoImplEx fr;
    ServoImplEx bl;
    ServoImplEx br;
    public static double FLservopos = .5;
    public static double FRservopos = .5;
    public static double BLservopos = .5;
    public static double BRservopos = .5;


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        efl = hardwareMap.get(AnalogInput.class, "eFL");
        fl = hardwareMap.get(ServoImplEx.class, "sFL");
        fl.setPwmRange(new PwmControl.PwmRange(505, 2495));

        efr = hardwareMap.get(AnalogInput.class, "eFR");
        fr = hardwareMap.get(ServoImplEx.class, "sFR");
        fr.setPwmRange(new PwmControl.PwmRange(505, 2495));

        ebl = hardwareMap.get(AnalogInput.class, "eBL");
        bl = hardwareMap.get(ServoImplEx.class, "sBL");
        bl.setPwmRange(new PwmControl.PwmRange(505, 2495));

        ebr = hardwareMap.get(AnalogInput.class, "eBR");
        br = hardwareMap.get(ServoImplEx.class, "sBR");
        br.setPwmRange(new PwmControl.PwmRange(505, 2495));

    }

    @Override
    public void loop() {

        double FLposition = efl.getVoltage() / 3.3 * 360;
        double FRposition = efr.getVoltage() / 3.3 * 360;
        double BLposition = ebl.getVoltage() / 3.3 * 360;
        double BRposition = ebr.getVoltage() / 3.3 * 360;


        // Telemetry
        telemetry.addData("1", "test");
        telemetry.addData("FL Position", FLposition);
        telemetry.addData("FR Position", FRposition);
        telemetry.addData("BL Position", BLposition);
        telemetry.addData("BR Position", BRposition);
        telemetry.update();

        if (gamepad1.a) {
            fl.setPosition(FLservopos);
            fr.setPosition(FRservopos);
            bl.setPosition(BLservopos);
            br.setPosition(BRservopos);

        }
    }
}
