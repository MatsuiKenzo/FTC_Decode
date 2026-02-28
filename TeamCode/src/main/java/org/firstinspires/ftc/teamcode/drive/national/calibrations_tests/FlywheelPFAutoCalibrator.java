package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Calibração automática inteligente de kP e kF do flywheel (Nacional – dois motores).
 *
 * Fase 1 – FeedForward (F): ajusta F com passos grandes quando o erro é grande (motor
 * longe do alvo) e passos menores quando se aproxima; se passar do alvo (overshoot),
 * reduz F. Igual ao tuner manual: “falta muito” → aumenta bastante.
 *
 * Fase 2 – Proporcional (P): com F razoável, ajusta P com passos adaptativos ao erro;
 * se oscilar, reduz P.
 *
 * Fase 3 – Refino: pequenos ajustes em P e F para minimizar MAE.
 *
 * Resultado: calibração mais rápida e estável. Copie KP e KF para ConstantsConf.Shooter.
 */
@TeleOp(name = "Flywheel PF Auto Calibrator", group = "Tuning")
public class FlywheelPFAutoCalibrator extends LinearOpMode {

    private static final double LOW_VELOCITY  = 1000.0;
    private static final double HIGH_VELOCITY = 1800.0;
    private static final int SETTLE_MS = 1200;
    private static final int SAMPLE_MS = 800;
    private static final int SAMPLE_DT_MS = 40;

    private static final double P_INIT = 0.0;   // Fase F começa com P=0
    private static final double F_INIT = 0.008;
    private static final double P_MIN = 0.0;
    private static final double P_MAX = 80.0;
    private static final double F_MIN = 0.001;
    private static final double F_MAX = 0.045;

    // Erro grande → passo grande; erro pequeno → passo pequeno
    private static final double ERROR_BIG   = 250.0;  // acima disso: passo grande
    private static final double ERROR_MID   = 80.0;   // entre mid e big: passo médio
    private static final double ERROR_GOOD  = 35.0;   // abaixo: refinamento
    private static final double F_STEP_BIG  = 0.015;
    private static final double F_STEP_MID  = 0.005;
    private static final double F_STEP_SMALL = 0.0012;
    private static final double P_STEP_BIG  = 18.0;
    private static final double P_STEP_MID  = 6.0;
    private static final double P_STEP_SMALL = 2.0;

    private static final int MAX_F_ITERATIONS = 20;
    private static final int MAX_P_ITERATIONS = 18;
    private static final int MAX_FINE_ITERATIONS = 12;

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;

    private enum Phase { F_COARSE, P_COARSE, FINE, DONE }
    private Phase phase = Phase.F_COARSE;
    private boolean calibrationDone = false;
    private double bestP = P_INIT;
    private double bestF = F_INIT;
    private double bestCost = Double.MAX_VALUE;
    private double currentP = P_INIT;
    private double currentF = F_INIT;
    private int iteration = 0;
    private int fPhaseIter = 0;
    private int pPhaseIter = 0;
    private int fineIter = 0;

    @Override
    public void runOpMode() {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_LEFT_MOTOR_NAME);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_RIGHT_MOTOR_NAME);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addLine("Flywheel PF Auto Calibrator (inteligente)");
        telemetry.addLine("START para iniciar. F -> P -> Refino.");
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

    /** Mede MAE e erro médio (signed) nas duas velocidades. meanError > 0 = abaixo do alvo. */
    private void sampleBothVelocities(SteadyStateResult out) {
        sampleSteadyState(HIGH_VELOCITY, out);
        double maeHigh = out.mae;
        double meanHigh = out.meanError;
        if (!opModeIsActive()) return;
        sampleSteadyState(LOW_VELOCITY, out);
        out.mae = maeHigh + out.mae;
        out.meanError = (meanHigh + out.meanError) / 2.0;
    }

    private static class SteadyStateResult {
        double mae;
        double meanError;
        double stdDev; // para detectar oscilação
    }

    private void sampleSteadyState(double targetVelocity, SteadyStateResult result) {
        setTargetVelocity(targetVelocity);
        long t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < SETTLE_MS) {
            sleep(SAMPLE_DT_MS);
        }
        double sumAbs = 0, sumSigned = 0, sumSq = 0;
        int n = 0;
        t0 = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - t0 < SAMPLE_MS) {
            double v = getCurrentVelocityAvg();
            double err = targetVelocity - v;
            sumAbs += Math.abs(err);
            sumSigned += err;
            sumSq += err * err;
            n++;
            sleep(SAMPLE_DT_MS);
        }
        result.mae = n > 0 ? sumAbs / n : 0;
        result.meanError = n > 0 ? sumSigned / n : 0;
        double mean = result.meanError;
        result.stdDev = n > 1 ? Math.sqrt(Math.max(0, (sumSq / n) - (mean * mean))) : 0;
    }

    private void runCalibrationStep() {
        SteadyStateResult r = new SteadyStateResult();

        if (phase == Phase.F_COARSE) {
            telemetry.addData("Fase", "FeedForward (F) it %d/%d", fPhaseIter + 1, MAX_F_ITERATIONS);
            telemetry.addData("F", "%.4f  (erro>0=aumenta F)", currentF);
            telemetry.update();
            applyPf(0, currentF); // P=0 para tunar só F
            sampleBothVelocities(r);
            if (!opModeIsActive()) return;

            double cost = r.mae;
            if (cost < bestCost) {
                bestCost = cost;
                bestP = 0;
                bestF = currentF;
            }

            if (cost <= ERROR_GOOD || fPhaseIter >= MAX_F_ITERATIONS) {
                phase = Phase.P_COARSE;
                currentP = 8.0; // começa P em valor razoável
                bestF = currentF;
                currentF = bestF;
                bestCost = Double.MAX_VALUE;
                pPhaseIter = 0;
                return;
            }

            double stepF;
            double absMean = Math.abs(r.meanError);
            if (absMean >= ERROR_BIG) stepF = F_STEP_BIG;
            else if (absMean >= ERROR_MID) stepF = F_STEP_MID;
            else stepF = F_STEP_SMALL;

            if (r.meanError > 0) {
                currentF = Math.min(F_MAX, currentF + stepF);
            } else {
                currentF = Math.max(F_MIN, currentF - stepF);
            }
            fPhaseIter++;
            return;
        }

        if (phase == Phase.P_COARSE) {
            telemetry.addData("Fase", "Proporcional (P) it %d/%d", pPhaseIter + 1, MAX_P_ITERATIONS);
            telemetry.addData("P | F", "%.2f | %.4f  (erro>0=aumenta P)", currentP, currentF);
            telemetry.update();
            applyPf(currentP, currentF);
            sampleBothVelocities(r);
            if (!opModeIsActive()) return;

            double cost = r.mae;
            if (cost < bestCost) {
                bestCost = cost;
                bestP = currentP;
                bestF = currentF;
            }

            if (cost <= ERROR_GOOD || pPhaseIter >= MAX_P_ITERATIONS) {
                phase = Phase.FINE;
                currentP = bestP;
                currentF = bestF;
                fineIter = 0;
                return;
            }

            // Oscilação: desvio alto em relação ao erro médio → reduz P
            if (r.stdDev > 1.5 * Math.abs(r.meanError) && r.stdDev > 40) {
                currentP = Math.max(P_MIN, currentP - P_STEP_SMALL);
                pPhaseIter++;
                return;
            }

            double stepP;
            double absMean = Math.abs(r.meanError);
            if (absMean >= ERROR_BIG) stepP = P_STEP_BIG;
            else if (absMean >= ERROR_MID) stepP = P_STEP_MID;
            else stepP = P_STEP_SMALL;

            if (r.meanError > 0) {
                currentP = Math.min(P_MAX, currentP + stepP);
            } else {
                currentP = Math.max(P_MIN, currentP - stepP);
            }
            pPhaseIter++;
            return;
        }

        if (phase == Phase.FINE) {
            telemetry.addData("Fase", "Refino it %d/%d", fineIter + 1, MAX_FINE_ITERATIONS);
            telemetry.addData("P | F", "%.2f | %.4f  cost=%.1f", currentP, currentF, bestCost);
            telemetry.update();
            applyPf(currentP, currentF);
            sampleBothVelocities(r);
            if (!opModeIsActive()) return;

            double cost = r.mae;
            if (cost < bestCost) {
                bestCost = cost;
                bestP = currentP;
                bestF = currentF;
            }

            if (fineIter >= MAX_FINE_ITERATIONS) {
                phase = Phase.DONE;
                calibrationDone = true;
                return;
            }

            // Refino: pequenos passos nos quatro vizinhos, escolhe o melhor
            double stepP = 2.0;
            double stepF = 0.0008;
            double bestLocal = cost;
            double np = currentP, nf = currentF;

            for (int dir = 0; dir < 4; dir++) {
                double tp = currentP, tf = currentF;
                if (dir == 0) tp = Math.min(P_MAX, currentP + stepP);
                else if (dir == 1) tp = Math.max(P_MIN, currentP - stepP);
                else if (dir == 2) tf = Math.min(F_MAX, currentF + stepF);
                else tf = Math.max(F_MIN, currentF - stepF);

                applyPf(tp, tf);
                sampleBothVelocities(r);
                if (!opModeIsActive()) return;
                if (r.mae < bestLocal) {
                    bestLocal = r.mae;
                    np = tp;
                    nf = tf;
                    if (r.mae < bestCost) {
                        bestCost = r.mae;
                        bestP = tp;
                        bestF = tf;
                    }
                }
            }

            if (bestLocal < cost) {
                currentP = np;
                currentF = nf;
            }
            fineIter++;
            return;
        }

        calibrationDone = true;
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
