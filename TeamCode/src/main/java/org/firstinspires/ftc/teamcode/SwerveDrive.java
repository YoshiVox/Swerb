package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp(name = "SwerveDrive")
public class SwerveDrive extends OpMode {
    // Constants
    private static final double VOLTAGE_MAX = 3.3;
    private static final double DEGREES_MAX = 360.0;
    private static final double SERVO_PWM_MIN = 505;
    private static final double SERVO_PWM_MAX = 2495;
    private static final double DEADZONE = 0.1;

    // Hardware Devices
    private AnalogInput[] encoders;
    private ServoImplEx[] servos;
    private DcMotor[] driveMotors;

    // Configuration variables (accessible via FTC Dashboard)
    // Encoder values (0-360) when wheels are pointing straight forward
    public static double FL_ZERO = 121.2;
    public static double FR_ZERO = 101.56;
    public static double BL_ZERO = 90.65;
    public static double BR_ZERO = 255.82;

    // Servo reversal configuration
    public static boolean FL_SERVO_REVERSED = true;  // Set these based on your robot
    public static boolean FR_SERVO_REVERSED = true;
    public static boolean BL_SERVO_REVERSED = false;
    public static boolean BR_SERVO_REVERSED = false;

    // Motor power scaling
    public static double DRIVE_POWER_SCALE = 0.8;
    public static double TURN_POWER_SCALE = 0.6;

    @Override
    public void init() {
        // Setup telemetry dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize arrays
        encoders = new AnalogInput[4];
        servos = new ServoImplEx[4];
        driveMotors = new DcMotor[4];

        // Initialize encoders
        String[] encoderNames = {"eFL", "eFR", "eBL", "eBR"};
        for (int i = 0; i < encoders.length; i++) {
            encoders[i] = hardwareMap.get(AnalogInput.class, encoderNames[i]);
        }

        // Initialize servos
        String[] servoNames = {"sFL", "sFR", "sBL", "sBR"};
        for (int i = 0; i < servos.length; i++) {
            servos[i] = hardwareMap.get(ServoImplEx.class, servoNames[i]);
            servos[i].setPwmRange(new PwmControl.PwmRange(SERVO_PWM_MIN, SERVO_PWM_MAX));
        }

        // Initialize drive motors
        String[] motorNames = {"FL", "FR", "BL", "BR"};
        for (int i = 0; i < driveMotors.length; i++) {
            driveMotors[i] = hardwareMap.get(DcMotor.class, motorNames[i]);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Init", "Swerve Drive Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get gamepad inputs
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Apply deadzone
        forward = Math.abs(forward) > DEADZONE ? forward : 0;
        strafe = Math.abs(strafe) > DEADZONE ? strafe : 0;
        rotate = Math.abs(rotate) > DEADZONE ? rotate : 0;

        // Calculate swerve drive values
        updateSwerveModules(forward, strafe, rotate);

        // Update telemetry
        updateTelemetry();
    }

    private double getEncoderDegrees(int index) {
        return encoders[index].getVoltage() / VOLTAGE_MAX * DEGREES_MAX;
    }

    private void updateSwerveModules(double forward, double strafe, double rotate) {
        if (Math.abs(forward) < DEADZONE && Math.abs(strafe) < DEADZONE && Math.abs(rotate) < DEADZONE) {
            // If no input, stop all motors but maintain module angles
            for (DcMotor motor : driveMotors) {
                motor.setPower(0);
            }
            return;
        }

        // Calculate wheel speeds and angles for swerve drive
        double r = Math.hypot(forward, strafe);
        double robotAngle = Math.atan2(strafe, forward);

        // Module zeros
        double[] zeros = {FL_ZERO, FR_ZERO, BL_ZERO, BR_ZERO};
        boolean[] reversedServos = {FL_SERVO_REVERSED, FR_SERVO_REVERSED, BL_SERVO_REVERSED, BR_SERVO_REVERSED};

        for (int i = 0; i < 4; i++) {
            // Add wheel-specific angle offset (0, π/2, π, 3π/2)
            double moduleAngle = robotAngle + (i * Math.PI/2);
            double targetAngle = Math.toDegrees(moduleAngle);

            // Normalize target angle to 0-360
            targetAngle = targetAngle % 360;
            if (targetAngle < 0) targetAngle += 360;

            // Get current module angle
            double currentAngle = getEncoderDegrees(i);

            // Calculate the difference between target and current angle
            double delta = targetAngle - currentAngle;

            // Normalize delta to -180 to 180
            while (delta > 180) delta -= 360;
            while (delta < -180) delta += 360;

            // Determine if we should flip the direction
            boolean shouldReverse = false;
            if (Math.abs(delta) > 90) {
                // If the target is more than 90 degrees away, flip the direction
                targetAngle = (targetAngle + 180) % 360;
                shouldReverse = true;
            }

            // Calculate module speed
            double speed = r * DRIVE_POWER_SCALE;
            if (rotate != 0) {
                speed = Math.max(Math.abs(speed), Math.abs(rotate * TURN_POWER_SCALE));
            }

            // Reverse speed if needed
            if (shouldReverse) {
                speed = -speed;
            }

            // Set motor power
            driveMotors[i].setPower(Range.clip(speed, -1.0, 1.0));

            // Convert target angle to servo position (0-1 range)
            double servoPos = (targetAngle - zeros[i]) / 180.0 + 0.5;

            // Apply servo reversal if configured
            if (reversedServos[i]) {
                servoPos = 1.0 - servoPos;
            }

            servoPos = Range.clip(servoPos, 0, 1);  // Ensure we stay within valid range
            servos[i].setPosition(servoPos);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Status", "Running");

        String[] moduleNames = {"FL", "FR", "BL", "BR"};
        double[] zeros = {FL_ZERO, FR_ZERO, BL_ZERO, BR_ZERO};
        boolean[] reversedServos = {FL_SERVO_REVERSED, FR_SERVO_REVERSED, BL_SERVO_REVERSED, BR_SERVO_REVERSED};

        for (int i = 0; i < encoders.length; i++) {
            double currentAngle = getEncoderDegrees(i);
            double servoPos = servos[i].getPosition();
            double motorPower = driveMotors[i].getPower();

            telemetry.addData(moduleNames[i] + " Module",
                    "Angle: %.1f° | Zero: %.1f° | Servo: %.3f | Power: %.2f | Reversed: %b",
                    currentAngle, zeros[i], servoPos, motorPower, reversedServos[i]);
        }

        telemetry.update();
    }
}