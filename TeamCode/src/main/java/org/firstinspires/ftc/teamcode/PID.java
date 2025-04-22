package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
public class PID extends OpMode {

    private DcMotorEx lift1;
    private PIDController controller;

    public static final double P = 0.005, I = 0, D = 0;


    static double target = 0;

    public double pid1;


    public void init() {
        lift1 = hardwareMap.get(DcMotorEx.class, "lift1");
        controller = new PIDController(P, I, D);
        lift1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {


        pid1 = controller.calculate(lift1.getCurrentPosition(), target);
        lift1.setPower(pid1);

    }
    }