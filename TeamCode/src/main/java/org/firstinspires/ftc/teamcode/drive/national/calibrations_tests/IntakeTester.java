package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * OpMode para testar apenas o intake e conferir se os dois motores giram no mesmo sentido
 * (sem shooter, turret, etc.). Use potência baixa (ex.: 0.2) para não danificar.
 *
 * Controles:
 * - D-pad CIMA/BAIXO: aumenta/diminui a potência (passo 0.05). Inicia em 0.20.
 * - A: roda os dois motores na potência selecionada (segure para rodar, solte para parar).
 * - Mesmas direções do IntakeSubsystem: intake = FORWARD, intake_2 = REVERSE
 *   (para os rolos puxarem no mesmo sentido).
 *
 * Robot Configuration: intake, intake_2 (opcional).
 */
@TeleOp(name = "Intake Tester", group = "Tuning")
public class IntakeTester extends OpMode {

    private DcMotorEx intakeMotor;
    private DcMotorEx intakeMotor2;

    /** Potência de teste (0 a 1). Ajuste com D-pad; começa baixa para segurança. */
    private double testPower = 0.20;
    private static final double POWER_STEP = 0.05;
    private static final double POWER_MIN = 0.0;
    private static final double POWER_MAX = 1.0;

    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    @Override
    public void init() {
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, ConstantsConf.Intake.INTAKE_MOTOR_NAME);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            intakeMotor = null;
        }
        try {
            intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake_2");
            intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            intakeMotor2 = null;
        }

        telemetry.addLine("Intake Tester - só os dois motores do intake");
        telemetry.addLine("D-pad U/D = potência | A = rodar (segure)");
        telemetry.addLine("Potência inicial: 0.20 (baixa para não danificar)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // D-pad: ajustar potência
        if (gamepad1.dpad_up && !dpadUpPrev) {
            testPower = Math.min(POWER_MAX, testPower + POWER_STEP);
        }
        if (gamepad1.dpad_down && !dpadDownPrev) {
            testPower = Math.max(POWER_MIN, testPower - POWER_STEP);
        }
        dpadUpPrev = gamepad1.dpad_up;
        dpadDownPrev = gamepad1.dpad_down;

        // A: rodar na potência selecionada (segurar = ligado)
        boolean run = gamepad1.a;
        double power = run ? testPower : 0.0;
        if (intakeMotor != null) intakeMotor.setPower(power);
        if (intakeMotor2 != null) intakeMotor2.setPower(power);

        telemetry.clear();
        telemetry.addLine("=== INTAKE TESTER ===");
        telemetry.addData("Potência", "%.2f (D-pad U/D)", testPower);
        telemetry.addData("Rodando?", run ? "SIM (A)" : "NÃO");
        telemetry.addData("Motor 1 (" + ConstantsConf.Intake.INTAKE_MOTOR_NAME + ")", intakeMotor != null ? "OK (FORWARD)" : "não encontrado");
        telemetry.addData("Motor 2 (intake_2)", intakeMotor2 != null ? "OK (REVERSE)" : "não encontrado");
        telemetry.addLine("Os dois devem puxar no mesmo sentido. Potência baixa = seguro.");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (intakeMotor != null) intakeMotor.setPower(0.0);
        if (intakeMotor2 != null) intakeMotor2.setPower(0.0);
    }
}
