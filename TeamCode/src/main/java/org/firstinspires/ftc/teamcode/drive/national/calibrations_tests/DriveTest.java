package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.national.objects.FieldOrientedDrive;

/**
 * OpMode MÍNIMO só para testar a movimentação (Field-Oriented Drive).
 *
 * Usa APENAS: FieldOrientedDrive. Sem Follower, Kalman, turret, shooter, intake.
 * Serve para isolar onde está o erro de movimentação (tremedeira, orientação, etc.).
 *
 * Robot Configuration: FL, FR, BL, BR (DcMotor), imu (IMU).
 *
 * Controles:
 * - Left stick: frente/trás e strafe (field-oriented)
 * - Right stick X: giro
 * - Left bumper: reset IMU (só na borda de subida, um clique = um reset)
 * Teste integração Nicolas no repositório
 */
@TeleOp(name = "Drive Test", group = "Tuning")
public class DriveTest extends OpMode {

    private FieldOrientedDrive fod;
    private boolean leftBumperPrev = false;

    @Override
    public void init() {
        fod = new FieldOrientedDrive(hardwareMap);
        telemetry.addLine("Drive Only Test - só FieldOrientedDrive");
        telemetry.addLine("Sticks = drive, LB = reset IMU (borda)");
        telemetry.addData("Config", "FL, FR, BL, BR, imu");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean lbNow = gamepad1.left_bumper;
        boolean resetIMUThisLoop = lbNow && !leftBumperPrev;
        leftBumperPrev = lbNow;

        fod.movement(
                -gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                resetIMUThisLoop
        );

        double headingDeg = fod.getHeadingDegrees();
        telemetry.clear();
        telemetry.addLine("=== SÓ MOVIMENTAÇÃO ===");
        telemetry.addData("Heading (IMU) °", "%.1f", headingDeg);
        telemetry.addData("Stick L (x,y)", "(%.2f, %.2f)", -gamepad1.left_stick_x, gamepad1.left_stick_y);
        telemetry.addData("Stick R x", "%.2f", gamepad1.right_stick_x);
        telemetry.addData("LB (reset IMU)", resetIMUThisLoop ? "RESET AGORA" : "solto");
        telemetry.update();
    }

    @Override
    public void stop() {
        // FieldOrientedDrive não expõe stop; motores param ao encerrar OpMode
    }
}
