package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.SwerveDriveSubsystem;

@TeleOp(name = "SwerveDriveOpMode")
public class SwerveDriveOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDriveSubsystem swerve = new SwerveDriveSubsystem(hardwareMap, telemetry);
        GamepadEx driver = new GamepadEx(gamepad1);

        // Set zero offsets for each pod if needed
        // Format: FL, FR, BL, BR
        swerve.setZeroOffsets(new double[]{0.0, 0.0, 0.0, 0.0});

        waitForStart();

        while (opModeIsActive()) {
            double y = -driver.getLeftY(); // Forward/back
            double x = driver.getLeftX();  // Strafe
            double rot = driver.getRightX(); // Rotation

            // Optional: Rotate field orientation using dpad
            if (driver.getButton(GamepadKeys.Button.DPAD_UP)) {
                swerve.setDriverOrientation(Rotation2d.fromDegrees(0));
            } else if (driver.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                swerve.setDriverOrientation(Rotation2d.fromDegrees(90));
            } else if (driver.getButton(GamepadKeys.Button.DPAD_DOWN)) {
                swerve.setDriverOrientation(Rotation2d.fromDegrees(180));
            } else if (driver.getButton(GamepadKeys.Button.DPAD_LEFT)) {
                swerve.setDriverOrientation(Rotation2d.fromDegrees(270));
            }

            swerve.drive(x, y, rot, true);
            swerve.periodic();

            telemetry.addData("Joystick X", x);
            telemetry.addData("Joystick Y", y);
            telemetry.addData("Rotation", rot);
            telemetry.update();
        }
    }
}
