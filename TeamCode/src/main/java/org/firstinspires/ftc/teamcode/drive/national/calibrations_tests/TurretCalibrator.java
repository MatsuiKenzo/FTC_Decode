package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.national.hardware.RobotHardwareNacional;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Calibração da mira da turret (relação engrenagem: graus por segundo por potência).
 *
 * A turret não tem encoder; o ângulo é estimado por potência × tempo × constante.
 * Se a constante estiver errada, a mira fica fora. Use este OpMode para medir a relação real.
 *
 * Controles:
 * - Stick X: gira a turret manualmente (vê ângulo estimado na telemetria)
 * - Y: zera ângulo estimado (considera torreta "para frente" = 0°)
 * - A: inicia corrida de 3 s a potência 0.5 (para medir)
 * - Após 3 s: use D-pad CIMA/BAIXO para ajustar "graus reais" (o que você mediu com transferidor)
 * - B: aplica o novo valor (recomendado = grausReais / 1.5) e grava em ConstantsConf
 * - D-pad ESQ/DIR: ajuste fino da constante atual (+/- 1) sem medir
 */
@TeleOp(name = "Turret Calibrator (mira)", group = "Tuning")
public class TurretCalibrator extends OpMode {

    private Follower follower;
    private RobotHardwareNacional robot;

    private static final Pose FIXED_POSE = new Pose(39, 80, Math.toRadians(180));
    private static final double MEASURE_POWER = 0.5;
    private static final double MEASURE_DURATION_SEC = 3.0;

    private ElapsedTime measureTimer = new ElapsedTime();
    private ElapsedTime loopTimer = new ElapsedTime();
    private boolean measuring = false;
    private double angleAtMeasureStart = 0.0;
    private double realDegreesEntered = 0.0;  // valor que o usuário ajusta com D-pad
    private boolean aPrev = false, bPrev = false, yPrev = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(FIXED_POSE);
        robot = new RobotHardwareNacional(hardwareMap, follower, true);
        if (robot.turret != null) {
            robot.turret.resetAngle(0.0);
        }
        loopTimer.reset();
        measureTimer.reset();
        telemetry.addLine("Turret Calibrator - calibrar relação engrenagem");
        telemetry.addLine("Stick X = girar | Y = zerar ângulo | A = medir 3s @ 0.5");
        telemetry.addLine("D-pad U/D = graus reais | B = aplicar e gravar");
        telemetry.addLine("D-pad L/R = ajuste fino constante +/-1");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (robot.turret == null) {
            telemetry.addLine("Turret nao disponivel.");
            telemetry.update();
            return;
        }

        // Y: zerar ângulo (torreta para frente = 0°)
        boolean yNow = gamepad1.y;
        if (yNow && !yPrev) {
            robot.turret.resetAngle(0.0);
        }
        yPrev = yNow;

        double dt = loopTimer.seconds();
        loopTimer.reset();

        // Modo: medindo 3 s OU controle manual
        if (measuring) {
            if (measureTimer.seconds() < MEASURE_DURATION_SEC) {
                robot.turret.setManualPowerAndIntegrate(MEASURE_POWER, dt);
            } else {
                robot.turret.stopRotation();
                measuring = false;
            }
        } else {
            double power = -gamepad1.left_stick_x;
            if (Math.abs(power) < 0.05) {
                robot.turret.stopRotation();
            } else {
                robot.turret.setManualPowerAndIntegrate(power, dt);
            }
        }

        // A: iniciar medição de 3 s
        boolean aNow = gamepad1.a;
        if (aNow && !aPrev && !measuring) {
            measuring = true;
            measureTimer.reset();
            angleAtMeasureStart = robot.turret.getMotorAngle();
        }
        aPrev = aNow;

        // D-pad: ajustar "graus reais" (após a medição) ou ajuste fino da constante
        if (gamepad1.dpad_up) realDegreesEntered += 1.0;
        if (gamepad1.dpad_down) realDegreesEntered -= 1.0;
        if (gamepad1.dpad_right) {
            double c = robot.turret.getDegreesPerSecondPerPower();
            robot.turret.setDegreesPerSecondPerPower(c + 1.0);
            ConstantsConf.Nacional.TURRET_DEGREES_PER_SECOND_PER_POWER = robot.turret.getDegreesPerSecondPerPower();
        }
        if (gamepad1.dpad_left) {
            double c = robot.turret.getDegreesPerSecondPerPower();
            robot.turret.setDegreesPerSecondPerPower(Math.max(1.0, c - 1.0));
            ConstantsConf.Nacional.TURRET_DEGREES_PER_SECOND_PER_POWER = robot.turret.getDegreesPerSecondPerPower();
        }

        // B: aplicar graus reais → novo constante e gravar
        boolean bNow = gamepad1.b;
        if (bNow && !bPrev) {
            if (realDegreesEntered != 0) {
                double newConstant = realDegreesEntered / (MEASURE_POWER * MEASURE_DURATION_SEC);
                newConstant = Math.max(1.0, Math.min(200.0, newConstant));
                robot.turret.setDegreesPerSecondPerPower(newConstant);
                ConstantsConf.Nacional.TURRET_DEGREES_PER_SECOND_PER_POWER = newConstant;
            }
        }
        bPrev = bNow;

        // Telemetria
        telemetry.clear();
        telemetry.addLine("=== TURRET CALIBRATOR ===");
        telemetry.addData("Ângulo est. (°)", "%.1f", robot.turret.getMotorAngle());
        telemetry.addData("Constante atual", "%.1f °/s por power", robot.turret.getDegreesPerSecondPerPower());

        if (measuring) {
            double elapsed = measureTimer.seconds();
            telemetry.addData("Medindo...", "%.1f s / %.1f s", elapsed, MEASURE_DURATION_SEC);
            if (elapsed >= MEASURE_DURATION_SEC) {
                double estimatedRot = robot.turret.getMotorAngle() - angleAtMeasureStart;
                telemetry.addData("Rotação estimada (°)", "%.1f", estimatedRot);
                telemetry.addLine("D-pad U/D = graus REAIS (transferidor), depois B = aplicar");
            }
        } else {
            telemetry.addData("Graus reais (D-pad)", "%.0f (depois B = aplicar)", realDegreesEntered);
            double recommended = realDegreesEntered / (MEASURE_POWER * MEASURE_DURATION_SEC);
            if (realDegreesEntered != 0) {
                telemetry.addData("Se aplicar (B)", "const = %.1f", recommended);
            }
        }
        telemetry.addLine("Y=zerar | A=medir 3s | D-pad L/R=fino +/-1");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (robot != null && robot.turret != null) {
            robot.turret.stopRotation();
        }
    }
}
