package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Calibração automática de kP e kF do flywheel (Nacional – dois motores).
 *
 * Comanda diretamente os dois motores do flywheel (shooter_left, shooter_right).
 * Alterna entre duas velocidades (baixa e alta), mede o MAE, ajusta P e F por
 * coordinate descent e exibe os melhores valores. Copie para ConstantsConf.Shooter (KP, KF).
 */
@TeleOp(name = "Flywheel PF Auto Calibrator", group = "Tuning")
public class FlywheelPFAutoCalibrator extends LinearOpMode {

    private static final double LOW_VELOCITY  = 1000.0;  // ticks/s
    private static final double HIGH_VELOCITY = 1800.0;   // ticks/s
    private static final int SETTLE_MS = 1200;
    private static final int SAMPLE_MS = 800;
    private static final int SAMPLE_DT_MS = 40;

    private static final double P_INIT = 25.0;
    private static final double F_INIT = 0.012;
    private static final double P_STEP_INIT = 8.0;
    private static final double F_STEP_INIT = 0.003;
    private static final double P_MIN = 2.0;
    private static final double P_MAX = 80.0;
    private static final double F_MIN = 0.002;
    private static final double F_MAX = 0.04;
    private static final int MAX_ITERATIONS = 25;

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;

    private boolean calibrationDone = false;
    private double bestP = P_INIT;
    private double bestF = F_INIT;
    private double bestCost = Double.MAX_VALUE;
    private double currentP = P_INIT;
    private double currentF = F_INIT;
    private double stepP = P_STEP_INIT;
    private double stepF = F_STEP_INIT;
    private int iteration = 0;

    @Override
    public void runOpMode() {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_LEFT_MOTOR_NAME);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_RIGHT_MOTOR_NAME);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Flywheel PF Auto Calibrator (2 motores)");
        telemetry.addLine("START para iniciar. Aguarde a calibração.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !calibrationDone) {
            runCalibrationStep();
            sleep(20);
        }

        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);

        while (opModeIsActive()) {
            showFinalTelemetry();
            sleep(100);
        }
    }

    private void applyPf(double p, double f) {
        PIDFCoefficients coef = new PIDFCoefficients(p, 0, 0, f);
        leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coef);
        rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coef);
    }

    private void setTargetVelocity(double velocity) {
        leftFlywheel.setVelocity(velocity);
        rightFlywheel.setVelocity(velocity);
    }

    private double getCurrentVelocityAvg() {
        return (leftFlywheel.getVelocity() + rightFlywheel.getVelocity()) / 2.0;
    }

    private void runCalibrationStep() {
        if (iteration >= MAX_ITERATIONS || (stepP < 1.0 && stepF < 0.0005)) {
            calibrationDone = true;
            return;
        }

        telemetry.addData("Fase", "Iteração %d | P=%.3f F=%.4f", iteration + 1, currentP, currentF);
        telemetry.addData("Melhor até agora", "P=%.3f F=%.4f cost=%.1f", bestP, bestF, bestCost);
        telemetry.update();

        applyPf(currentP, currentF);

        double costHigh = sampleMae(HIGH_VELOCITY);
        if (!opModeIsActive()) return;
        double costLow = sampleMae(LOW_VELOCITY);
        if (!opModeIsActive()) return;

        double cost = costHigh + costLow;
        if (cost < bestCost) {
            bestCost = cost;
            bestP = currentP;
            bestF = currentF;
        }

        double bestNeighborCost = cost;
        double nextP = currentP;
        double nextF = currentF;

        double tryP = Math.min(P_MAX, currentP + stepP);
        if (tryP != currentP) {
            applyPf(tryP, currentF);
            double c = sampleMae(HIGH_VELOCITY) + sampleMae(LOW_VELOCITY);
            if (c < bestNeighborCost) { bestNeighborCost = c; nextP = tryP; nextF = currentF; }
            if (c < bestCost) { bestCost = c; bestP = tryP; bestF = currentF; }
            if (!opModeIsActive()) return;
        }
        tryP = Math.max(P_MIN, currentP - stepP);
        if (tryP != currentP) {
            applyPf(tryP, currentF);
            double c = sampleMae(HIGH_VELOCITY) + sampleMae(LOW_VELOCITY);
            if (c < bestNeighborCost) { bestNeighborCost = c; nextP = tryP; nextF = currentF; }
            if (c < bestCost) { bestCost = c; bestP = tryP; bestF = currentF; }
            if (!opModeIsActive()) return;
        }
        double tryF = Math.min(F_MAX, currentF + stepF);
        if (tryF != currentF) {
            applyPf(currentP, tryF);
            double c = sampleMae(HIGH_VELOCITY) + sampleMae(LOW_VELOCITY);
            if (c < bestNeighborCost) { bestNeighborCost = c; nextP = currentP; nextF = tryF; }
            if (c < bestCost) { bestCost = c; bestP = currentP; bestF = tryF; }
            if (!opModeIsActive()) return;
        }
        tryF = Math.max(F_MIN, currentF - stepF);
        if (tryF != currentF) {
            applyPf(currentP, tryF);
            double c = sampleMae(HIGH_VELOCITY) + sampleMae(LOW_VELOCITY);
            if (c < bestNeighborCost) { bestNeighborCost = c; nextP = currentP; nextF = tryF; }
            if (c < bestCost) { bestCost = c; bestP = currentP; bestF = tryF; }
            if (!opModeIsActive()) return;
        }

        if (bestNeighborCost < cost) {
            currentP = nextP;
            currentF = nextF;
        } else {
            stepP *= 0.5;
            stepF *= 0.5;
            currentP = bestP;
            currentF = bestF;
        }
        iteration++;
    }

    private double sampleMae(double targetVelocity) {
        setTargetVelocity(targetVelocity);
        long t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < SETTLE_MS) {
            sleep(SAMPLE_DT_MS);
        }
        double sumAbsError = 0;
        int n = 0;
        t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < SAMPLE_MS) {
            double avg = getCurrentVelocityAvg();
            sumAbsError += Math.abs(targetVelocity - avg);
            n++;
            sleep(SAMPLE_DT_MS);
        }
        return n > 0 ? sumAbsError / n : 0;
    }

    private void showFinalTelemetry() {
        telemetry.clear();
        telemetry.addLine(">>> CALIBRAÇÃO CONCLUÍDA <<<");
        telemetry.addLine("");
        telemetry.addData("Melhor KP", "%.4f", bestP);
        telemetry.addData("Melhor KF", "%.4f", bestF);
        telemetry.addData("Custo (MAE)", "%.1f", bestCost);
        telemetry.addLine("");
        telemetry.addLine("Copie para ConstantsConf.Shooter:");
        telemetry.addData("  KP", "public static double KP = %.4f;", bestP);
        telemetry.addData("  KF", "public static double KF = %.4f;", bestF);
        telemetry.update();
    }
}
