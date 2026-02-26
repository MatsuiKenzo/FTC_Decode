package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.national.hardware.RobotHardwareNacional;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Testa o encoder da torreta (ex.: REV Through Bore V1).
 * Ative TURRET_ENCODER_ENABLED = true em ConstantsConf.Nacional e cadastre "turret_encoder"
 * na Robot Configuration (porta de encoder de um canal; motor desse canal desconectado).
 *
 * Controles:
 * - Stick X: gira a turret manualmente
 * - Y: define zero (posição atual = 0°)
 *
 * Telemetria: ângulo (°), ticks brutos, zero do encoder, se está usando encoder.
 */

@TeleOp(name = "Turret Encoder Tester", group = "Tuning")
public class TurretEncoderTester extends OpMode {

    private Follower follower;
    private RobotHardwareNacional robot;
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean yPrev = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        robot = new RobotHardwareNacional(hardwareMap, follower, true);
        if (robot.turret != null) {
            robot.turret.resetAngle(0.0);
        }
        loopTimer.reset();
        telemetry.addLine("Turret Encoder Tester");
        telemetry.addLine("Stick X = girar | Y = zerar (0°)");
        telemetry.addLine("Ative TURRET_ENCODER_ENABLED e configure turret_encoder no config.");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (robot.turret == null) {
            telemetry.addLine("Turret nao disponivel.");
            telemetry.update();
            return;
        }

        boolean yNow = gamepad1.y;
        if (yNow && !yPrev) {
            robot.turret.resetAngle(0.0);
        }
        yPrev = yNow;

        double dt = loopTimer.seconds();
        loopTimer.reset();
        double power = -gamepad1.left_stick_x;
        if (Math.abs(power) < 0.05) {
            robot.turret.stopRotation();
        } else {
            robot.turret.setManualPowerAndIntegrate(power, dt);
        }

        telemetry.clear();
        telemetry.addLine("=== TURRET ENCODER TEST ===");
        telemetry.addData("Usando encoder?", robot.turret.isUsingEncoder() ? "Sim" : "Nao");
        telemetry.addData("Ângulo (°)", "%.2f", robot.turret.getMotorAngle());
        telemetry.addData("Ticks brutos", robot.turret.getEncoderPositionRaw());
        telemetry.addData("Zero (ticks)", robot.turret.getEncoderZeroPosition());
        telemetry.addLine("Y = zerar (0°)");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot != null && robot.turret != null) {
            robot.turret.stopRotation();
        }
    }
}
