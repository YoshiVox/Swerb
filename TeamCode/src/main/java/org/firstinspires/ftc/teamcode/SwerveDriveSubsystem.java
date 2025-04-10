package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SwerveDriveSubsystem extends SubsystemBase {

    public static class SwerveModuleState {
        public double speedMetersPerSecond;
        public Rotation2d angle;

        public SwerveModuleState(double speedMetersPerSecond, Rotation2d angle) {
            this.speedMetersPerSecond = speedMetersPerSecond;
            this.angle = angle;
        }
    }

    private static final double MAX_VOLTAGE = 3.3;
    private static final double MAX_SERVO_DEGREES = 355.0;

    private final SwervePod[] pods = new SwervePod[4];
    private final Translation2d[] modulePositions;
    private Rotation2d driverOrientation = new Rotation2d();

    public SwerveDriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        // Define swerve module positions (x, y in meters)
        modulePositions = new Translation2d[] {
                new Translation2d(0.16, 0.16),   // frontLeft
                new Translation2d(0.16, -0.16),  // frontRight
                new Translation2d(-0.16, 0.16),  // backLeft
                new Translation2d(-0.16, -0.16)  // backRight
        };

        pods[0] = new SwervePod("fl", hardwareMap, 0);
        pods[1] = new SwervePod("fr", hardwareMap, 1);
        pods[2] = new SwervePod("bl", hardwareMap, 2);
        pods[3] = new SwervePod("br", hardwareMap, 3);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldOriented) {
        Pose2d chassisSpeeds;
        if (fieldOriented) {
            double cosA = driverOrientation.getCos();
            double sinA = driverOrientation.getSin();

            double x = xSpeed * cosA - ySpeed * sinA;
            double y = xSpeed * sinA + ySpeed * cosA;
            chassisSpeeds = new Pose2d();
        } else {
            chassisSpeeds = new Pose2d();
        }

        SwerveModuleState[] states = toSwerveModuleStates(chassisSpeeds);

        for (int i = 0; i < pods.length; i++) {
            pods[i].setDesiredState(states[i]);
        }
    }

    public void setZeroOffsets(double[] offsets) {
        for (int i = 0; i < 4; i++) {
            pods[i].setZeroOffset(offsets[i]);
        }
    }

    public void setDriverOrientation(Rotation2d orientation) {
        this.driverOrientation = orientation;
    }

    @Override
    public void periodic() {
        for (SwervePod pod : pods) {
            pod.update();
        }
    }

    private SwerveModuleState[] toSwerveModuleStates(Pose2d chassisSpeeds) {
        double vx = chassisSpeeds.getX();
        double vy = chassisSpeeds.getY();
        double omega = chassisSpeeds.getHeading();

        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i = 0; i < modulePositions.length; i++) {
            Translation2d module = modulePositions[i];
            double rx = -omega * module.getY();
            double ry = omega * module.getX();

            double vxr = vx + rx;
            double vyr = vy + ry;

            double speed = Math.hypot(vxr, vyr);
            double angle = Math.toDegrees(Math.atan2(vyr, vxr));

            states[i] = new SwerveModuleState(speed, Rotation2d.fromDegrees(angle));
        }

        return states;
    }

    private static class SwervePod {
        private final DcMotorEx driveMotor;
        private final ServoImplEx turningServo;
        private final AnalogInput encoder;
        private final PIDController pid;

        private double zeroOffset;
        private final int id;

        public SwervePod(String name, HardwareMap hardwareMap, int id) {
            this.driveMotor = hardwareMap.get(DcMotorEx.class, name + "_motor");
            this.turningServo = hardwareMap.get(ServoImplEx.class, name + "_servo");
            this.encoder = hardwareMap.get(AnalogInput.class, name + "_encoder");
            this.id = id;

            turningServo.setPwmRange(new PwmControl.PwmRange(505, 2495));
            this.pid = new PIDController(0.015, 0.0, 0.001);
            this.zeroOffset = 0;
        }

        public void setZeroOffset(double offset) {
            this.zeroOffset = offset;
        }

        public void setDesiredState(SwerveModuleState state) {
            double targetAngle = state.angle.getDegrees();
            double currentAngle = getAbsoluteAngle();

            double delta = targetAngle - currentAngle;
            delta = wrapDelta(delta);

            if (Math.abs(delta) > 90) {
                delta = wrapDelta(delta + 180);
                state = new SwerveModuleState(-state.speedMetersPerSecond, Rotation2d.fromDegrees(targetAngle + 180));
            }

            double output = pid.calculate(currentAngle, state.angle.getDegrees());
            driveMotor.setPower(state.speedMetersPerSecond);
            turningServo.setPosition((output + 1.0) / 2.0);
        }

        public void update() {
            // No periodic update logic for now
        }

        private double getAbsoluteAngle() {
            double raw = (encoder.getVoltage() / MAX_VOLTAGE) * 360;
            double angle = raw - zeroOffset;
            if (angle < 0) angle += 360;
            return angle;
        }

        private double wrapDelta(double delta) {
            while (delta > 180) delta -= 360;
            while (delta < -180) delta += 360;
            return delta;
        }
    }

    private static class PIDController {
        private final double kP, kI, kD;
        private double prevError = 0, integral = 0;

        public PIDController(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }

        public double calculate(double current, double target) {
            double error = target - current;
            error = (error + 180) % 360 - 180;

            integral += error;
            double derivative = error - prevError;
            prevError = error;

            return kP * error + kI * integral + kD * derivative;
        }
    }
}
