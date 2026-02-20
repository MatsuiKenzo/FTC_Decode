package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.qualifiers.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Test OpMode for shooter subsystem.
 *
 * Controls:
 * - A button: Set to low velocity
 * - B button: Set to medium velocity
 * - X button: Set to high velocity
 * - Y button: Stop shooter
 *
 * Telemetry shows:
 * - Target velocity
 * - Current velocity
 * - Velocity error
 * - Ready status
 * - Battery voltage
 * - Voltage compensation factor
 */
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTest extends LinearOpMode {

    private RobotHardware robot;

    @Override
    public void runOpMode() {
        // Initialize hardware
        robot = new RobotHardware(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "A=Low, B=Med, X=High, Y=Stop");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update subsystems
            robot.update();

            // Control shooter
            if (gamepad1.a) {
                robot.shooter.setTargetVelocity(ConstantsConf.Shooter.LOW_VELOCITY);
            } else if (gamepad1.b) {
                robot.shooter.setTargetVelocity(ConstantsConf.Shooter.MEDIUM_VELOCITY);
            } else if (gamepad1.x) {
                robot.shooter.setTargetVelocity(ConstantsConf.Shooter.HIGH_VELOCITY);
            } else if (gamepad1.y) {
                robot.shooter.stop();
            }

            // Display telemetry
            telemetry.addData("Target RPM", "%.0f", robot.shooter.getTargetRPM());
            telemetry.addData("Current RPM", "%.0f", robot.shooter.getCurrentRPM());
            telemetry.addData("RPM Error", "%.0f", robot.shooter.getVelocityErrorRPM());
            telemetry.addData("Ready", robot.shooter.isReady() ? "YES" : "NO");
            telemetry.addData("Battery Voltage", "%.2f V", robot.shooter.getVoltage());
            telemetry.addData("Voltage Compensation", "%.3f", robot.shooter.getVoltageCompensation());
            telemetry.addLine();
            telemetry.addData("Controls", "A=Low, B=Med, X=High, Y=Stop");
            telemetry.update();
        }

        // Stop everything
        robot.stop();
    }
}
