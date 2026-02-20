package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.qualifiers.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Example TeleOp
 *
 * This demonstrates:
 * - Velocity control for consistent shooting
 * - Waiting for shooter to be ready before shooting
 * - Battery compensation maintaining consistent power
 *
 * Controls:
 * - Right Trigger: Spin up shooter to high velocity
 * - Right Bumper: Shoot (only works when shooter is ready)
 * - Left Trigger: Spin up shooter to low velocity
 * - Y button: Stop shooter
 */
@TeleOp(name = "Example TeleOp", group = "Example")
public class ExampleTeleOp extends LinearOpMode {

    private RobotHardware robot;
    private boolean shooterWasReady = false;

    @Override
    public void runOpMode() {
        robot = new RobotHardware(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update subsystems (important - runs the PID control loop)
            robot.update();

            // Haptic feedback when shooter becomes ready
            boolean shooterReady = robot.shooter.isReady();
            if (shooterReady && !shooterWasReady) {
                // Short rumble to indicate "ready to shoot"
                gamepad1.rumble(1.0, 1.0, 250);
            }
            shooterWasReady = shooterReady;

            // Control shooter
            if (gamepad1.right_trigger > 0.5) {
                // Spin up to high velocity
                robot.shooter.setTargetVelocity(ConstantsConf.Shooter.HIGH_VELOCITY);
            } else if (gamepad1.left_trigger > 0.5) {
                // Spin up to low velocity
                robot.shooter.setTargetVelocity(ConstantsConf.Shooter.LOW_VELOCITY);
            } else if (gamepad1.y) {
                // Stop shooter
                robot.shooter.stop();
            }

            // Shoot
            if (gamepad1.right_bumper) {
                robot.intake.setPower(1.0);
                sleep(800); // Shoot for 200ms
                robot.intake.setPower(0.0);
            }

            // Display telemetry
            telemetry.addData("Shooter Status", "");
            telemetry.addData("  Target", "%.0f", robot.shooter.getTargetVelocity());
            telemetry.addData("  Current", "%.0f", robot.shooter.getCurrentVelocity());
            telemetry.addData("  Ready", robot.shooter.isReady() ? "YES ✓" : "NO ✗");
            telemetry.addData("  Battery", "%.2f V", robot.shooter.getVoltage());
            telemetry.addLine();
            telemetry.addData("Controls", "RT=High, LT=Low, RB=Shoot, Y=Stop");
            telemetry.update();
        }

        robot.stop();
    }
}
