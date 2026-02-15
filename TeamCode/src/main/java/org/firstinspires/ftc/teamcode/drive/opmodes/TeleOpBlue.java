package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.drive.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * TeleOpBlue
 *
 * Features:
 * - Field-oriented drive
 * - Velocity control com PID no shooter
 * - Compensação automática de bateria
 * - Turret automática usando PedroPathing
 * - Intake com detecção de bola
 *
 * Controls:
 * - Gamepad 1: Drive (left stick), Turn (right stick X), Reset IMU (left bumper)
 * - Gamepad 1: Reset pose (B), Recalibrar goal = onde está mirando (Y), Travar turret (A = toggle)
 * - Gamepad 1: Shoot (right trigger)
 * - Gamepad 1: Intake (left trigger + right trigger)
 */
@TeleOp(name = "TeleOp Blue", group = "Refactored")
public class TeleOpBlue extends OpMode {
    private FieldOrientedDrive fod;
    private RobotHardware robot;
    private Follower follower;
    private boolean shooterWasReady = false;
    private boolean turretLocked = false;
    private boolean aPrev = false;
    private boolean yPrev = false;

    // Starting pose for blue alliance
    private final Pose startTeleop = new Pose(39, 80, Math.toRadians(180));

    // Target position (blue goal)
    private double targetX = 6.0;
    private double targetY = 140.0;

    @Override
    public void init() {
        // Initialize field-oriented drive
        fod = new FieldOrientedDrive(hardwareMap);

        // Initialize PedroPathing
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startTeleop);

        // Initialize robot hardware with PedroPathing integration
        robot = new RobotHardware(hardwareMap, follower);

        // Set target position
        robot.setTargetPosition(targetX, targetY);

        telemetry.addData("Status", "Inicializado. Pedro Pathing Ativo.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Update PedroPathing
        follower.update();

        // Update subsystems (PID control)
        robot.update();

        // Haptic feedback when shooter becomes ready
        boolean shooterReady = robot.shooter.isReady();
        if (shooterReady && !shooterWasReady) {
            // Short rumble to indicate "ready to shoot"
            gamepad1.rumble(1.0, 1.0, 250);
        }
        shooterWasReady = shooterReady;

        // Field-oriented drive
        fod.movement(
                -gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                gamepad1.left_bumper
        );

        // Intake control
        robot.intake.collect(-gamepad1.left_trigger, -gamepad1.right_trigger);

        // Shoot control
        robot.intake.shoot(gamepad1.right_trigger);

        // Reset pose (B)
        if (gamepad1.b) {
            follower.setPose(startTeleop);
        }

        // Recalibrar goal = onde está mirando agora (Y) — só o ângulo, mantém a distância
        boolean yNow = gamepad1.y;
        if (yNow && !yPrev) {
            Pose robotPose = follower.getPose();
            double dist = robot.shooter.getDistance();
            double headingRad = robotPose.getHeading();
            double turretDeg = robot.turret.getMotorAngle();
            double absoluteAngleRad = headingRad + Math.toRadians(turretDeg);
            double newTargetX = robotPose.getX() + dist * Math.cos(absoluteAngleRad);
            double newTargetY = robotPose.getY() + dist * Math.sin(absoluteAngleRad);
            robot.setTargetPosition(newTargetX, newTargetY);
        }
        yPrev = yNow;

        // Lock turret angle (toggle: um clique trava, outro destrava)
        boolean aNow = gamepad1.a;
        if (aNow && !aPrev) {
            turretLocked = !turretLocked;
        }
        aPrev = aNow;
        if (turretLocked) {
            robot.turret.lockAngle(-45.0);
        } else {
            robot.turret.unlockAngle();
        }

        // Telemetry
        telemetry.addData("--- Shooter ---", "");
        telemetry.addData("Target Velocity", "%.0f", robot.shooter.getTargetVelocity());
        telemetry.addData("Current Velocity", "%.0f", robot.shooter.getCurrentVelocity());
        telemetry.addData("Ready", robot.shooter.isReady() ? "YES ✓" : "NO ✗");
        telemetry.addData("Distance", "%.1f pol", robot.shooter.getDistance());
        telemetry.addData("Battery", "%.2f V", robot.shooter.getVoltage());

        telemetry.addData("--- Turret ---", "");
        telemetry.addData("Angle", "%.1f°", robot.turret.getMotorAngle());
        telemetry.addData("Travada (A)", turretLocked ? "SIM" : "NÃO");
        telemetry.addData("Y", "Recalibrar goal = onde está mirando");

        telemetry.addData("--- Pose ---", "");
        telemetry.addData("X", "%.1f", follower.getPose().getX());
        telemetry.addData("Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("--- Intake ---", "");
        telemetry.addData("Has Ball", robot.intake.hasBall() ? "YES" : "NO");
        telemetry.addData("Ball Color", robot.intake.getBallColor());

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
