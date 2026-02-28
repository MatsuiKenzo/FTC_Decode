package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * TeleOp para testar os servos Tauron da pá (flap).
 * Suporta dois servos no flap; ambos giram juntos na mesma posição.
 *
 * Controles:
 * - A: posição 0.0 (padrão / fechado)
 * - B: posição 1.0 (alinhado com shooter)
 * - D-Pad Up/Down: ajuste fino da posição (+/- 0.05)
 * - X: posição 0.5 (meio)
 * - Right Trigger: controla proporcionalmente a potência do motor de intake/indexer
 *
 * Configure os servos como "flap" e "flap2" no Robot Configuration.
 */
@TeleOp(name = "Programador Taura", group = "Tuning")
public class TauraProgrammer extends OpMode {

    private Servo flapServo;
    private Servo flapServo2;
    private double currentPosition = 0.0;

    // Motor de intake/indexer (mesmo motor do IntakeSubsystem)
    private DcMotorEx intakeMotor;

    private static final double POS_MIN = 0.0;
    private static final double POS_MAX = 1.0;
    private static final double STEP = 0.05;

    @Override
    public void init() {
        try {
            flapServo = hardwareMap.get(Servo.class, ConstantsConf.Intake.FLAP_SERVO_NAME);
            flapServo.setPosition(POS_MAX);
            currentPosition = POS_MAX;
            telemetry.addData("Status", "Servo '%s' encontrado.", ConstantsConf.Intake.FLAP_SERVO_NAME);
        } catch (Exception e) {
            flapServo = null;
            telemetry.addData("Erro", "Servo '%s' nao encontrado.", ConstantsConf.Intake.FLAP_SERVO_NAME);
        }
        try {
            flapServo2 = hardwareMap.get(Servo.class, ConstantsConf.Intake.FLAP2_SERVO_NAME);
            flapServo2.setPosition(POS_MAX);
            telemetry.addData("Status2", "Servo '%s' encontrado (gira junto).", ConstantsConf.Intake.FLAP2_SERVO_NAME);
        } catch (Exception e) {
            flapServo2 = null;
            telemetry.addData("Info", "Servo '%s' nao encontrado (opcional).", ConstantsConf.Intake.FLAP2_SERVO_NAME);
        }

        // Motor de intake/indexer para testar junto com o flap
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, ConstantsConf.Intake.INTAKE_MOTOR_NAME);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            telemetry.addData("Status Motor", "Motor de intake/indexer encontrado: %s", ConstantsConf.Intake.INTAKE_MOTOR_NAME);
        } catch (Exception e) {
            intakeMotor = null;
            telemetry.addData("Erro Motor", "Motor de intake nao encontrado (%s).", ConstantsConf.Intake.INTAKE_MOTOR_NAME);
        }
        telemetry.addLine();
        telemetry.addData("A", "Posicao 0.0 (padrao)");
        telemetry.addData("B", "Posicao 1.0 (alinhado)");
        telemetry.addData("X", "Posicao 0.5 (meio)");
        telemetry.addData("D-Pad Up/Down", "Ajuste fino +/- 0.05");
        telemetry.addData("Right Trigger", "Potencia intake/indexer 0.0 -> 1.0");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (flapServo == null && flapServo2 == null) {
            telemetry.addData("Status", "Nenhum servo do flap disponivel.");
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
            sleep(200);
        }
        if (gamepad1.dpad_down) {
            currentPosition = Math.max(POS_MIN, currentPosition - STEP);
            sleep(200);
        }

        // Os dois servos giram juntos na mesma posição
        if (flapServo != null) flapServo.setPosition(currentPosition);
        if (flapServo2 != null) flapServo2.setPosition(currentPosition);

        // Gatilho direito controla proporcionalmente o motor de intake/indexer
        // Se estiver girando ao contrário, inverta o sinal (como abaixo)
        if (intakeMotor != null) {
            // Inverte o sentido para ficar na mesma direção do intake desejado
            double power = -gamepad1.right_trigger; // 0.0 a -1.0
            intakeMotor.setPower(power);
        }

        telemetry.addLine("");

        telemetry.addData("Posicao atual", "%.2f (0.0 = padrao, 1.0 = alinhado)", currentPosition);
        if (flapServo != null) {
            telemetry.addData("Servo getPosition()", "%.2f", flapServo.getPosition());
        }
        if (flapServo2 != null) {
            telemetry.addData("Servo2 getPosition()", "%.2f", flapServo2.getPosition());
        }
        if (intakeMotor != null) {
            telemetry.addData("Intake Power (RT)", "%.2f", gamepad1.right_trigger);
        } else {
            telemetry.addData("Intake Power (RT)", "Motor nao disponivel");
        }
        telemetry.addLine();
        telemetry.addData("A/B/X", "0.0 / 1.0 / 0.5");
        telemetry.addData("D-Pad", "ajuste fino +/- 0.05");
        telemetry.addData("Right Trigger", "potencia intake/indexer 0.0 -> 1.0");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (flapServo != null) flapServo.setPosition(POS_MIN);
        if (flapServo2 != null) flapServo2.setPosition(POS_MIN);
        if (intakeMotor != null) intakeMotor.setPower(0.0);
    }
}




