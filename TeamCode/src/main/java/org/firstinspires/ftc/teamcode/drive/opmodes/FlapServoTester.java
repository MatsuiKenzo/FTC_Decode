package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TeleOp para testar o servo Tauron da pá (flap).
 *
 * Controles:
 * - A: posição 0.0 (padrão / fechado)
 * - B: posição 1.0 (alinhado com shooter)
 * - D-Pad Up/Down: ajuste fino da posição (+/- 0.05)
 * - X: posição 0.5 (meio)
 *
 * Configure o servo como "flap" no Robot Configuration.
 */
@TeleOp(name = "Flap Servo Tester", group = "Tuning")
public class FlapServoTester extends OpMode {

    private Servo flapServo;
    private double currentPosition = 0.0;

    private static final double POS_MIN = 0.0;
    private static final double POS_MAX = 1.0;
    private static final double STEP = 0.05;

    @Override
    public void init() {
        try {
            flapServo = hardwareMap.get(Servo.class, "flap");
            flapServo.setPosition(POS_MIN);
            currentPosition = POS_MIN;
            telemetry.addData("Status", "Servo 'flap' encontrado. Pronto para testar.");
        } catch (Exception e) {
            flapServo = null;
            telemetry.addData("Erro", "Servo 'flap' nao encontrado. Verifique o nome no Robot Configuration.");
        }
        telemetry.addLine();
        telemetry.addData("A", "Posicao 0.0 (padrao)");
        telemetry.addData("B", "Posicao 1.0 (alinhado)");
        telemetry.addData("X", "Posicao 0.5 (meio)");
        telemetry.addData("D-Pad Up/Down", "Ajuste fino +/- 0.05");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (flapServo == null) {
            telemetry.addData("Status", "Servo nao disponivel.");
            telemetry.update();
            return;
        }

        // Botões para posições fixas
        if (gamepad1.a) {
            currentPosition = POS_MIN;
        }
        if (gamepad1.b) {
            currentPosition = POS_MAX;
        }
        if (gamepad1.x) {
            currentPosition = 0.5;
        }

        // D-Pad para ajuste fino
        if (gamepad1.dpad_up) {
            currentPosition = Math.min(POS_MAX, currentPosition + STEP);
        }
        if (gamepad1.dpad_down) {
            currentPosition = Math.max(POS_MIN, currentPosition - STEP);
        }

        flapServo.setPosition(currentPosition);

        telemetry.addData("Posicao atual", "%.2f (0.0 = padrao, 1.0 = alinhado)", currentPosition);
        telemetry.addData("Servo getPosition()", "%.2f", flapServo.getPosition());
        telemetry.addLine();
        telemetry.addData("A", "0.0 | B = 1.0 | X = 0.5 | D-Pad = ajuste");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (flapServo != null) {
            flapServo.setPosition(POS_MIN);
        }
    }
}
