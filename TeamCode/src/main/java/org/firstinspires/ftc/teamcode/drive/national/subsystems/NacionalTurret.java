package org.firstinspires.ftc.teamcode.drive.national.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Turret nacional: dois CRServos contínuos (mesma potência nos dois) + encoder.
 *
 * Hardware:
 * - leftServo, rightServo: giram juntos no mesmo sentido físico
 * - Encoder (ex.: REV Through Bore) em um canal (ConstantsConf.Nacional.TURRET_ENCODER_*)
 *
 * Comportamento:
 * - Aponta para o alvo (targetX, targetY) em coordenadas de campo usando a pose do Follower
 * - Ângulo da turret = 0° = frente do robô (convenção do Follower)
 * - Limites [minLimit, maxLimit] em graus; fora disso faz clip (não wrap)
 * - PID com deadband; timer resetado todo loop. Só opera com encoder configurado.
 */
public class NacionalTurret {
    private Follower follower;
    private CRServo leftServo;
    private CRServo rightServo;
    private DcMotorEx turretEncoder;
    private int encoderZeroPosition = 0;
    private double encoderTicksPerTurretRev;
    private double encoderDirection = 1.0;

    private double minLimit = -120.0;
    private double maxLimit = 120.0;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private boolean isLocked = false;
    private double lockedAngle = 0.0;
    private double currentAngle = 0.0;

    private double kP = 0.006;
    private double kI = 0.0;
    private double kD = 0.0005;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private final ElapsedTime timer = new ElapsedTime();

    // -------------------------------------------------------------------------
    // Init
    // -------------------------------------------------------------------------

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
    }

    // -------------------------------------------------------------------------
    // Setters públicos (alvo, limites, trava)
    // -------------------------------------------------------------------------

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

    // -------------------------------------------------------------------------
    // Update: lê encoder, calcula alvo, PID, aplica potência
    // -------------------------------------------------------------------------

    public void update() {
        if (turretEncoder == null) {
            stopRotation();
            timer.reset();
            return;
        }
        currentAngle = rawEncoderToDegrees();

        double targetDegrees;
        if (isLocked) {
            targetDegrees = lockedAngle;
        } else {
            if (follower == null) {
                stopRotation();
                timer.reset();
                return;
            }
            targetDegrees = computeTargetAngle();
        }

        targetDegrees = Range.clip(targetDegrees, minLimit, maxLimit);
        double power = computePID(targetDegrees, currentAngle);
        setRotationPower(power);
        timer.reset();
    }

    /**
     * Ângulo em graus que a turret deve apontar (referência do robô: 0 = frente).
     * Vector robô → alvo, ângulo no campo, menos heading do robô.
     */
    private double computeTargetAngle() {
        Pose pose = follower.getPose();
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double angleToTargetDeg = Math.toDegrees(Math.atan2(dy, dx));
        double robotHeadingDeg = Math.toDegrees(pose.getHeading());
        double relativeDeg = angleToTargetDeg - robotHeadingDeg;
        while (relativeDeg > 180) relativeDeg -= 360;
        while (relativeDeg < -180) relativeDeg += 360;
        return relativeDeg;
    }

    /** Normaliza ângulo para [-180, 180]. */
    private static double normalizeAngle(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    /**
     * PID com deadband. Erro normalizado para caminho mais curto.
     * Timer resetado no final do update() para dt correto no próximo loop.
     */
    private double computePID(double target, double current) {
        double error = normalizeAngle(target - current);
        double deadband = ConstantsConf.Nacional.TURRET_DEADBAND_DEG;
        if (Math.abs(error) <= deadband) {
            integralSum *= 0.5;
            lastError = error;
            return 0.0;
        }
        double dt = timer.seconds();
        if (dt <= 0 || dt > 1.0) {
            return 0.0;
        }
        if (Math.abs(error) > 90) integralSum *= 0.5;
        integralSum += error * dt;
        integralSum = Range.clip(integralSum, -0.5, 0.5);
        double derivative = (error - lastError) / dt;
        lastError = error;
        timer.reset();
        double out = kP * error + kI * integralSum + kD * derivative;
        return Range.clip(out, -1.0, 1.0);
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

    // -------------------------------------------------------------------------
    // Controle manual e parada
    // -------------------------------------------------------------------------

    public void setManualPower(double power) {
        setRotationPower(power);
    }

    /** Controle manual (ângulo vem do encoder). Parâmetro dtSeconds ignorado. */
    public void setManualPowerAndIntegrate(double power, double dtSeconds) {
        setRotationPower(power);
    }

    public void stopRotation() {
        if (leftServo != null) leftServo.setPower(0.0);
        if (rightServo != null) rightServo.setPower(0.0);
    }

    // -------------------------------------------------------------------------
    // Getters e utilitários
    // -------------------------------------------------------------------------

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
            // Queremos que a posição física atual seja interpretada como 'angle' graus.
            // (raw - zero) * 360 / encoderTicksPerTurretRev * encoderDirection = angle
            int deltaTicks = (int) Math.round(angle * encoderTicksPerTurretRev / 360.0 / encoderDirection);
            encoderZeroPosition = turretEncoder.getCurrentPosition() - deltaTicks;
        }
        currentAngle = angle;
        integralSum = 0.0;
        lastError = 0.0;
        timer.reset();
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

    /** Valor em ConstantsConf (mantido para compatibilidade com TurretCalibrator). */
    public double getDegreesPerSecondPerPower() {
        return ConstantsConf.Nacional.TURRET_DEGREES_PER_SECOND_PER_POWER;
    }

    /** Atualiza ConstantsConf (mantido para compatibilidade com TurretCalibrator). */
    public void setDegreesPerSecondPerPower(double value) {
        ConstantsConf.Nacional.TURRET_DEGREES_PER_SECOND_PER_POWER = value;
    }
}
