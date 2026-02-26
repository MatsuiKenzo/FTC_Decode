package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.national.hardware.RobotHardwareNacional;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * OpMode para testar a base giratória (turret) isoladamente.
 *
 * Robot Configuration: turret_left, turret_right (CRServo), imu, pinpoint (Follower precisa).
 *
 * Controles:
 * - Left stick X: gira a base manualmente (modo manual)
 * - A: alterna entre modo MANUAL (stick) e AUTO (turret aponta para o goal)
 *
 * Init: pose igual TeleOp Blue (39, 80, 180°); torreta considerada "para frente" (ângulo 0°).
 */

@TeleOp(name = "Turret Tester (Base giratória)", group = "Tuning")
public class TurretTester extends OpMode {

    private Follower follower;
    private RobotHardwareNacional robot;

    private static final Pose FIXED_POSE = new Pose(39, 80, Math.toRadians(180));
    private static final double GOAL_X = 6.0;
    private static final double GOAL_Y = 140.0;

    private boolean manualMode = true;
    private boolean aPrev = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(FIXED_POSE);

        robot = new RobotHardwareNacional(hardwareMap, follower, true);
        robot.setTargetPosition(GOAL_X, GOAL_Y);

        // Torreta "para frente" = ângulo 0° em relação ao robô (referência ao iniciar)
        if (robot.turret != null) {
            robot.turret.resetAngle(0.0);
        }

        telemetry.addLine("Turret Tester - Base giratória");
        telemetry.addData("Modo", manualMode ? "MANUAL (stick X)" : "AUTO (aponta goal)");
        telemetry.addData("A", "Alternar modo");
        telemetry.addData("Pose", "(39, 80, 180°) = igual TeleOp Blue");
        telemetry.addData("Torreta", "0° = para frente");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean aNow = gamepad1.a;
        if (aNow && !aPrev) {
            manualMode = !manualMode;
            if (!manualMode) {
                follower.setPose(FIXED_POSE);
            }
        }
        aPrev = aNow;

        if (robot.turret == null) {
            telemetry.addLine("ERRO: Turret nao inicializada. Confira turret_left e turret_right.");
            telemetry.update();
            return;
        }

        if (manualMode) {
            double power = -gamepad1.left_stick_x;
            if (Math.abs(power) < 0.05) {
                robot.turret.stopRotation();
            } else {
                robot.turret.setManualPower(power);
            }
        } else {
            follower.update();
            robot.updateWithoutShooter();
        }

        telemetry.clear();
        telemetry.addLine("=== BASE GIRATÓRIA (TURRET) ===");
        telemetry.addData("Modo", manualMode ? "MANUAL" : "AUTO (goal)");
        telemetry.addData("Ângulo est. (°)", "%.1f", robot.turret.getMotorAngle());
        telemetry.addData("A", "Alternar modo");
        if (manualMode) {
            telemetry.addData("Stick X", "Girar (%.2f)", -gamepad1.left_stick_x);
        } else {
            telemetry.addData("Pose", "(%.1f, %.1f)", follower.getPose().getX(), follower.getPose().getY());
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot != null && robot.turret != null) {
            robot.turret.stopRotation();
        }
    }
}
