package org.firstinspires.ftc.teamcode.drive.shooternacional;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shooter nacional com DOIS motores de flywheel.
 *
 * Não está conectado em nenhum TeleOp/Autônomo ainda.
 * Use esta classe apenas em testes separados.
 */
public class NacionalShooter {

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private VoltageSensor voltageSensor;

    // Alvo e estado
    private double targetVelocity = 0.0;      // ticks/s
    private double currentVelocityLeft = 0.0; // ticks/s
    private double currentVelocityRight = 0.0;// ticks/s
    private boolean isReady = false;

    private final ElapsedTime readyTimer = new ElapsedTime();
    private static final double READY_TOLERANCE = 50.0; // tolerância de RPM
    private static final double READY_TIME = 0.2;       // segundos dentro da tolerância

    // Ganhos PIDF básicos (ajuste depois nos testes)
    private double kP = 40.0;
    private double kI = 0.000001;
    private double kD = 0.0001;
    private double kF = 15.0;

    // Estado do PID
    private double integralLeft = 0.0;
    private double lastErrorLeft = 0.0;
    private double integralRight = 0.0;
    private double lastErrorRight = 0.0;
    private double lastTime = 0.0;

    // Compensação de bateria
    private static final double NOMINAL_VOLTAGE = 12.0;
    private double voltageCompensation = 1.0;

    /**
     * Inicializa o shooter nacional com dois motores.
     *
     * @param hardwareMap HardwareMap do OpMode
     * @param leftName    nome do motor esquerdo no config
     * @param rightName   nome do motor direito no config
     */
    public NacionalShooter(HardwareMap hardwareMap, String leftName, String rightName) {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, leftName);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, rightName);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Sensor de tensão (usa o primeiro disponível)
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        resetPID();
    }

    private void resetPID() {
        integralLeft = 0.0;
        lastErrorLeft = 0.0;
        integralRight = 0.0;
        lastErrorRight = 0.0;
        lastTime = System.nanoTime() / 1e9;
        isReady = false;
        readyTimer.reset();
    }

    /** Define a velocidade alvo (ticks por segundo) para os DOIS flywheels. */
    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
        isReady = false;
        readyTimer.reset();
    }

    /** Define PIDF em tempo de execução (para testes). */
    public void setPID(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        resetPID();
    }

    /**
     * Atualiza o controle PIDF dos dois motores.
     * Chame no loop do OpMode.
     */
    public void update() {
        // Velocidades atuais
        currentVelocityLeft = leftFlywheel.getVelocity();
        currentVelocityRight = rightFlywheel.getVelocity();

        // Compensação de tensão
        double currentVoltage = voltageSensor.getVoltage();
        voltageCompensation = NOMINAL_VOLTAGE / currentVoltage;

        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;
        if (dt <= 0 || dt > 1.0) dt = 0.02; // fallback ~20ms

        // LEFT
        double errorLeft = targetVelocity - currentVelocityLeft;
        integralLeft += errorLeft * dt;
        // anti-windup simples
        double maxIntegral = 0.5 / Math.max(kI, 1e-9);
        integralLeft = Math.max(-maxIntegral, Math.min(maxIntegral, integralLeft));
        double derivativeLeft = (errorLeft - lastErrorLeft) / dt;

        double outputLeft = (kP * errorLeft + kI * integralLeft + kD * derivativeLeft + kF * targetVelocity) * voltageCompensation;
        outputLeft = Math.max(-1.0, Math.min(1.0, outputLeft));

        // RIGHT
        double errorRight = targetVelocity - currentVelocityRight;
        integralRight += errorRight * dt;
        integralRight = Math.max(-maxIntegral, Math.min(maxIntegral, integralRight));
        double derivativeRight = (errorRight - lastErrorRight) / dt;

        double outputRight = (kP * errorRight + kI * integralRight + kD * derivativeRight + kF * targetVelocity) * voltageCompensation;
        outputRight = Math.max(-1.0, Math.min(1.0, outputRight));

        // Aplica potências
        leftFlywheel.setPower(outputLeft);
        rightFlywheel.setPower(outputRight);

        lastErrorLeft = errorLeft;
        lastErrorRight = errorRight;
        lastTime = currentTime;

        // Estado \"ready\" (média das velocidades)
        double avgError = (Math.abs(errorLeft) + Math.abs(errorRight)) / 2.0;
        if (avgError < READY_TOLERANCE) {
            if (!isReady) {
                readyTimer.reset();
            }
            isReady = readyTimer.seconds() >= READY_TIME;
        } else {
            isReady = false;
            readyTimer.reset();
        }
    }

    /** Para completamente os dois flywheels. */
    public void stop() {
        targetVelocity = 0.0;
        leftFlywheel.setPower(0.0);
        rightFlywheel.setPower(0.0);
        resetPID();
    }

    // Getters de apoio para telemetria

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrentVelocityLeft() {
        return currentVelocityLeft;
    }

    public double getCurrentVelocityRight() {
        return currentVelocityRight;
    }

    public boolean isReady() {
        return isReady;
    }

    public double getVoltage() {
        return voltageSensor != null ? voltageSensor.getVoltage() : 0.0;
    }
}

