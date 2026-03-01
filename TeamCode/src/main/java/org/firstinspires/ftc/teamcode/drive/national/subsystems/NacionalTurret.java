package org.firstinspires.ftc.teamcode.drive.national.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

public class NacionalTurret {
    private Follower follower;
    private CRServo leftServo;
    private CRServo rightServo;
    private DcMotorEx turretEncoder;

    private int encoderZeroPosition = 0;
    private double encoderTicksPerTurretRev;
    private double encoderDirection = 1.0;

    private double minLimit = -180.0;
    private double maxLimit = 180.0;

    // Alvo em coordenadas de campo
    private double targetX = 0.0;
    private double targetY = 0.0;

    private boolean isLocked = false;
    private double lockedAngle = 0.0;

    private double currentAngle = 0.0;

    // PID
    private double kP = 0.006;
    private double kI = 0.0;
    private double kD = 0.0005;

    private double integralSum = 0.0;
    private double lastError = 0.0;
    private final ElapsedTime timer = new ElapsedTime();

    // Anti-hunting / suavização
    private double holdDeadbandDeg = ConstantsConf.Nacional.TURRET_DEADBAND_DEG;

    // Potência mínima para vencer atrito
    private double minEffectivePower = 0.06;

    // Limita mudança brusca de potência
    private double maxPowerStepPerUpdate = 0.12; // 0.08–0.18
    private double lastPower = 0.0;

    // Unwind
    private double unwindMarginDeg = 12.0; // 8–20 típico

    // Correção de eixos
    private boolean swapXY = false;
    private boolean invertX = false;
    private boolean invertY = false;

    // ---------- Debug (telemetria temporária) ----------
    private double debugLastTargetDeg = 0.0;
    private double debugLastError = 0.0;
    private double debugLastPowerRaw = 0.0;
    private double debugLastPowerOut = 0.0;
    private double debugLastAngleToTargetDeg = 0.0; // antes do clip (computeTargetAngle)
    private double debugCurrentForPID = 0.0;         // ângulo enrolado [-180, 180] usado no PID

    // Init

    public void init(HardwareMap hardwareMap, Follower follower, String leftServoName, String rightServoName) {
        this.follower = follower;
        leftServo = hardwareMap.get(CRServo.class, leftServoName);
        rightServo = hardwareMap.get(CRServo.class, rightServoName);

        if (ConstantsConf.Nacional.TURRET_ENCODER_ENABLED) {
            try {
                turretEncoder = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.TURRET_ENCODER_MOTOR_NAME);
                turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

                encoderZeroPosition = turretEncoder.getCurrentPosition();

                int ticksPerRev = ConstantsConf.Nacional.TURRET_ENCODER_TICKS_PER_REV;
                int turretTeeth = ConstantsConf.Nacional.TURRET_ENCODER_GEAR_TURRET_TEETH;
                int encoderTeeth = ConstantsConf.Nacional.TURRET_ENCODER_GEAR_ENCODER_TEETH;

                encoderTicksPerTurretRev = (double) ticksPerRev * turretTeeth / encoderTeeth;
                encoderDirection = ConstantsConf.Nacional.TURRET_ENCODER_DIRECTION;
            } catch (Exception e) {
                turretEncoder = null;
            }
        } else {
            turretEncoder = null;
        }

        stopRotation();
        timer.reset();
        integralSum = 0.0;
        lastError = 0.0;
        currentAngle = 0.0;
        lastPower = 0.0;
    }

    // Setters públicos

    public void setTargetPosition(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public void setLimits(double min, double max) {
        this.minLimit = min;
        this.maxLimit = max;
    }

    public void lockAngle(double angle) {
        this.lockedAngle = angle;
        this.isLocked = true;
    }

    public void unlockAngle() {
        this.isLocked = false;
    }


    // Update: lê encoder, calcula alvo, PID, aplica potência

    public void update() {
        if (turretEncoder == null) {
            stopRotation();
            timer.reset();
            return;
        }

        currentAngle = rawEncoderToDegrees();
        // Usar ângulo "enrolado" em [-180, 180] no PID para o erro não explodir após várias voltas
        double currentForPID = normalizeAngle(currentAngle);
        debugCurrentForPID = currentForPID;

        double targetDegrees;
        if (isLocked) {
            targetDegrees = lockedAngle;
            debugLastAngleToTargetDeg = lockedAngle;
        } else {
            if (follower == null) {
                stopRotation();
                timer.reset();
                return;
            }
            targetDegrees = computeTargetAngle();
            debugLastAngleToTargetDeg = targetDegrees;
        }

        // Respeita limites físicos
        targetDegrees = Range.clip(targetDegrees, minLimit, maxLimit);
        debugLastTargetDeg = targetDegrees;

        // PID -> potência (com ângulo enrolado para erro em escala normal)
        double powerRaw = computePID(targetDegrees, currentForPID);
        debugLastPowerRaw = powerRaw;

        // Slew-rate
        double power = applySlewRate(powerRaw);
        debugLastPowerOut = power;

        setRotationPower(power);
    }

    /**
     * Ângulo em graus que a turret deve apontar (referência do robô: 0 = frente).
     * Vector robô → alvo, ângulo no campo, menos heading do robô.
     */
    private double computeTargetAngle() {
        Pose pose = follower.getPose();

        double rx = pose.getX();
        double ry = pose.getY();

        double tx = targetX;
        double ty = targetY;

        if (swapXY) {
            double tmp = tx; tx = ty; ty = tmp;
            tmp = rx; rx = ry; ry = tmp;
        }
        if (invertX) { tx = -tx; rx = -rx; }
        if (invertY) { ty = -ty; ry = -ry; }

        double dx = tx - rx;
        double dy = ty - ry;

        double angleToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());

        // Ângulo da turret relativo ao robô: convenção invertida para corrigir no sentido oposto ao giro do robô
        double relativeDeg = robotHeadingDeg - angleToTargetDeg;
        relativeDeg = normalizeAngle(relativeDeg);

        return relativeDeg;
    }

    /** Normaliza ângulo para [-180, 180]. */
    private static double normalizeAngle(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    /**
     * Erro para o PID: caminho que permaneça dentro de [minLimit, maxLimit].
     * Se o caminho mais curto passaria do limite, usa o outro sentido.
     */
    private double computeSmartError(double targetDeg, double currentDeg) {
        double error = normalizeAngle(targetDeg - currentDeg);
        double pathEnd = currentDeg + error;

        if (pathEnd > maxLimit) {
            error -= 360;
        } else if (pathEnd < minLimit) {
            error += 360;
        }
        return error;
    }

    /**
     * PID com deadband + unwind + potência mínima.
     * Timer é resetado aqui.
     */
    private double computePID(double target, double current) {
        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0 || dt > 0.5) {
            // dt absurdo -> não confia em derivada / integral
            integralSum = 0.0;
            lastError = 0.0;
            return 0.0;
        }

        // Erro inteligente (curto ou unwind perto dos limites)
        double error = computeSmartError(target, current);
        debugLastError = error;

        // Deadband/hold (mata o "caçar" quando parado)
        double deadband = holdDeadbandDeg;
        if (Math.abs(error) <= deadband) {
            integralSum *= 0.3;
            lastError = error;
            return 0.0;
        }

        // Integral com anti-windup
        integralSum += error * dt;
        integralSum = Range.clip(integralSum, -0.5, 0.5);

        // Derivada
        double derivative = (error - lastError) / dt;
        lastError = error;

        double out = (kP * error) + (kI * integralSum) + (kD * derivative);
        out = Range.clip(out, -1.0, 1.0);

        // Potência mínima só quando longe do alvo; perto do alvo evita overshoot/oscilação
        double minPowerThresholdDeg = holdDeadbandDeg * 2.5;
        if (Math.abs(error) > minPowerThresholdDeg && Math.abs(out) > 0 && Math.abs(out) < minEffectivePower) {
            out = Math.signum(out) * minEffectivePower;
        }

        return out;
    }

    private double applySlewRate(double desiredPower) {
        double delta = desiredPower - lastPower;
        delta = Range.clip(delta, -maxPowerStepPerUpdate, maxPowerStepPerUpdate);
        lastPower = Range.clip(lastPower + delta, -1.0, 1.0);
        return lastPower;
    }

    private double rawEncoderToDegrees() {
        if (turretEncoder == null || encoderTicksPerTurretRev <= 0) return 0.0;
        int raw = turretEncoder.getCurrentPosition() - encoderZeroPosition;
        return (raw * 360.0 / encoderTicksPerTurretRev) * encoderDirection;
    }

    private void setRotationPower(double power) {
        if (leftServo == null || rightServo == null) return;
        power = Range.clip(power, -1.0, 1.0);
        leftServo.setPower(power);
        rightServo.setPower(power);
    }


    // Controle manual e parada

    public void setManualPower(double power) {
        // Manual também respeita slew pra não dar tranco
        power = Range.clip(power, -1.0, 1.0);
        power = applySlewRate(power);
        setRotationPower(power);
    }

    /** Controle manual (ângulo vem do encoder). */
    public void setManualPowerAndIntegrate(double power, double dtSeconds) {
        setManualPower(power);
    }

    public void stopRotation() {
        if (leftServo != null) leftServo.setPower(0.0);
        if (rightServo != null) rightServo.setPower(0.0);
        lastPower = 0.0;
    }


    // Getters e utilitários

    public double getMotorAngle() {
        return turretEncoder != null ? rawEncoderToDegrees() : 0.0;
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void resetAngle(double angle) {
        if (turretEncoder != null) {
            int deltaTicks = (int) Math.round(angle * encoderTicksPerTurretRev / 360.0 / encoderDirection);
            encoderZeroPosition = turretEncoder.getCurrentPosition() - deltaTicks;
        }
        currentAngle = angle;
        integralSum = 0.0;
        lastError = 0.0;
        timer.reset();
        lastPower = 0.0;
    }

    public boolean isUsingEncoder() {
        return turretEncoder != null;
    }

    public int getEncoderPositionRaw() {
        return turretEncoder != null ? turretEncoder.getCurrentPosition() : -1;
    }

    public int getEncoderZeroPosition() {
        return encoderZeroPosition;
    }

    /** Valor em ConstantsConf*/
    public double getDegreesPerSecondPerPower() {
        return ConstantsConf.Nacional.TURRET_DEGREES_PER_SECOND_PER_POWER;
    }

    /** Atualiza ConstantsConf */
    public void setDegreesPerSecondPerPower(double value) {
        ConstantsConf.Nacional.TURRET_DEGREES_PER_SECOND_PER_POWER = value;
    }

    // ---------- Debug: telemetria temporária para diagnosticar turret ----------
    public double getDebugLastTargetDeg() { return debugLastTargetDeg; }
    public double getDebugLastError() { return debugLastError; }
    public double getDebugLastPowerRaw() { return debugLastPowerRaw; }
    public double getDebugLastPowerOut() { return debugLastPowerOut; }
    public double getDebugLastAngleToTargetDeg() { return debugLastAngleToTargetDeg; }
    public double getDebugLimitsMin() { return minLimit; }
    public double getDebugLimitsMax() { return maxLimit; }
    /** Ângulo enrolado em [-180, 180] usado no PID (evita erro gigante). */
    public double getDebugCurrentAngleWrapped() { return debugCurrentForPID; }

    // -------------------------------------------------------------------------
    // Extras opcionais (não quebram nada; só se você quiser usar)
    // -------------------------------------------------------------------------

    /** Se continuar “só vertical”, teste ligar isso em runtime pra validar convenção do eixo. */
    public void setAxisFix(boolean swapXY, boolean invertX, boolean invertY) {
        this.swapXY = swapXY;
        this.invertX = invertX;
        this.invertY = invertY;
    }

    public void setUnwindConfig(double marginDeg) {
        this.unwindMarginDeg = Math.max(0.0, marginDeg);
    }

    public void setAntiHuntConfig(double deadbandDeg, double minPower, double maxStep) {
        this.holdDeadbandDeg = Math.max(0.0, deadbandDeg);
        this.minEffectivePower = Range.clip(minPower, 0.0, 1.0);
        this.maxPowerStepPerUpdate = Range.clip(maxStep, 0.01, 1.0);
    }
}  