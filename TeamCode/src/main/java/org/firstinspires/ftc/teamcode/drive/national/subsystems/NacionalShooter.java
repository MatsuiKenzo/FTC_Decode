package org.firstinspires.ftc.teamcode.drive.national.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;
import org.firstinspires.ftc.teamcode.drive.util.ShooterDistanceToRPM;

/**
 * Shooter nacional com DOIS motores de flywheel.
 * 
 * Implementação completa baseada no PedroPathingShooter, mas com dois motores.
 * 
 * Features:
 * - Velocity PID control para velocidade consistente do flywheel (ambos os motores)
 * - Compensação automática de bateria
 * - Ajuste automático de velocidade baseado na distância ao alvo (usando PedroPathing)
 * - Feedforward para resposta rápida
 * - Estado "ready" para garantir tiros precisos (baseado na média dos dois motores)
 * 
 * Integrado com PedroPathing para calcular distância ao alvo e ajustar velocidade automaticamente.
 */
public class NacionalShooter {
    private Follower follower;
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private VoltageSensor voltageSensor;

    // Controle de velocidade
    private double targetVelocity = 0.0;
    private double currentVelocityLeft = 0.0;
    private double currentVelocityRight = 0.0;
    private double currentVelocityAvg = 0.0;
    private boolean isReady = false;

    // Estado do alvo
    private double targetX = 0.0;
    private double targetY = 0.0;

    // Configuração de potência baseada em distância
    private double minPower = 0.35;
    private double maxPower = 0.9;
    private double minDistance = 20.0; // polegadas
    private double maxDistance = 120.0; // polegadas

    // Configuração de velocidade (será calculada baseado na potência)
    private double minVelocity = ConstantsConf.Shooter.LOW_VELOCITY;
    private double maxVelocity = ConstantsConf.Shooter.HIGH_VELOCITY;

    // PID coefficients
    private double kP = ConstantsConf.Shooter.KP;
    private double kI = ConstantsConf.Shooter.KI;
    private double kD = ConstantsConf.Shooter.KD;
    private double kF = ConstantsConf.Shooter.KF;

    // PID state - um conjunto para cada motor
    private double integralLeft = 0.0;
    private double lastErrorLeft = 0.0;
    private double integralRight = 0.0;
    private double lastErrorRight = 0.0;
    private double lastTime = 0.0;

    // Battery compensation
    private double voltageCompensation = 1.0;

    // Ready detection
    private ElapsedTime readyTimer = new ElapsedTime();
    private static final double READY_TOLERANCE = 50.0; // RPM tolerance
    private static final double READY_TIME = 0.2; // seconds

    /** Se true, update() recalcula target por distância ao alvo; se false, só usa o RPM/velocidade definida manualmente. */
    private boolean useDistanceBasedVelocity = true;

    /** LUT distância → RPM (interpolação linear própria). */
    private final ShooterDistanceToRPM distanceToRPM = new ShooterDistanceToRPM();

    /**
     * Initialize the nacional shooter subsystem with PedroPathing integration.
     *
     * @param hardwareMap HardwareMap from OpMode
     * @param follower PedroPathing Follower instance (can be null)
     * @param leftMotorName Name of the left flywheel motor in hardware map
     * @param rightMotorName Name of the right flywheel motor in hardware map
     */
    public NacionalShooter(HardwareMap hardwareMap, Follower follower, String leftMotorName, String rightMotorName) {
        this.follower = follower;
        this.leftFlywheel = hardwareMap.get(DcMotorEx.class, leftMotorName);
        this.rightFlywheel = hardwareMap.get(DcMotorEx.class, rightMotorName);

        // Configure motors for velocity control
        leftFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        leftFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        
        // Left REVERSE (gira ao contrário do right); right FORWARD
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        // Get voltage sensor
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        resetPID();
    }

    /**
     * Set the target position for distance-based velocity calculation.
     *
     * @param x Target X coordinate (inches)
     * @param y Target Y coordinate (inches)
     */
    public void setTargetPosition(double x, double y) {
        this.targetX = x;
        this.targetY = y;
        updateTargetVelocityFromDistance();
    }

    /**
     * Set power configuration for distance-based control.
     *
     * @param minP Minimum power (at minDistance)
     * @param maxP Maximum power (at maxDistance)
     * @param minD Minimum distance (inches)
     * @param maxD Maximum distance (inches)
     */
    public void setPowerConfig(double minP, double maxP, double minD, double maxD) {
        this.minPower = minP;
        this.maxPower = maxP;
        this.minDistance = minD;
        this.maxDistance = maxD;
        updateTargetVelocityFromDistance();
    }

    /**
     * Set target velocity directly (overrides distance-based calculation).
     *
     * @param velocity Target velocity in ticks per second (para ambos os motores)
     */
    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
        isReady = false;
        readyTimer.reset();
    }

    /**
     * Set target velocity from RPM.
     *
     * @param rpm Target speed in revolutions per minute
     */
    public void setTargetRPM(double rpm) {
        setTargetVelocity(rpmToTicksPerSecond(rpm));
    }

    /**
     * Ativa ou desativa o cálculo de velocidade por distância ao alvo.
     * Em false, o shooter obedece apenas ao RPM/velocidade definido por setTargetRPM/setTargetVelocity.
     */
    public void setUseDistanceBasedVelocity(boolean use) {
        this.useDistanceBasedVelocity = use;
    }

    /**
     * Get current speed in RPM (média dos dois motores).
     *
     * @return Current velocity in revolutions per minute
     */
    public double getCurrentRPM() {
        return ticksPerSecondToRPM(currentVelocityAvg);
    }

    /**
     * Get target speed in RPM.
     *
     * @return Target velocity in revolutions per minute
     */
    public double getTargetRPM() {
        return ticksPerSecondToRPM(targetVelocity);
    }

    /**
     * Get velocity error in RPM.
     *
     * @return Target minus current, in RPM
     */
    public double getVelocityErrorRPM() {
        return getTargetRPM() - getCurrentRPM();
    }

    private double ticksPerSecondToRPM(double ticksPerSecond) {
        if (ConstantsConf.Shooter.TICKS_PER_REVOLUTION <= 0) return 0;
        return ticksPerSecond / ConstantsConf.Shooter.TICKS_PER_REVOLUTION * 60.0;
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * ConstantsConf.Shooter.TICKS_PER_REVOLUTION / 60.0;
    }

    /**
     * Update the shooter PID control loop.
     * Call this in your OpMode's loop() method.
     */
    public void update() {
        if (useDistanceBasedVelocity) {
            updateTargetVelocityFromDistance();
        }

        // Get current velocities
        currentVelocityLeft = leftFlywheel.getVelocity();
        currentVelocityRight = rightFlywheel.getVelocity();
        currentVelocityAvg = (currentVelocityLeft + currentVelocityRight) / 2.0;

        // Calculate voltage compensation
        double currentVoltage = voltageSensor.getVoltage();
        voltageCompensation = ConstantsConf.Shooter.NOMINAL_VOLTAGE / currentVoltage;

        // Calculate time delta
        double currentTime = System.nanoTime() / 1e9;
        double deltaTime = currentTime - lastTime;

        if (deltaTime == 0 || deltaTime > 1.0) {
            deltaTime = 0.02; // Default to 20ms
        }

        // LEFT MOTOR PID
        double errorLeft = targetVelocity - currentVelocityLeft;
        double proportionalLeft = kP * errorLeft;
        
        integralLeft += errorLeft * deltaTime;
        double maxIntegral = 0.5 / (kI > 0 ? kI : 1.0);
        integralLeft = Math.max(-maxIntegral, Math.min(maxIntegral, integralLeft));
        double integralTermLeft = kI * integralLeft;
        
        double derivativeLeft = (errorLeft - lastErrorLeft) / deltaTime;
        double derivativeTermLeft = kD * derivativeLeft;
        
        double feedforwardLeft = kF * targetVelocity;
        
        double outputLeft = (proportionalLeft + integralTermLeft + derivativeTermLeft + feedforwardLeft) * voltageCompensation;
        outputLeft = Math.max(-1.0, Math.min(1.0, outputLeft));

        // RIGHT MOTOR PID
        double errorRight = targetVelocity - currentVelocityRight;
        double proportionalRight = kP * errorRight;
        
        integralRight += errorRight * deltaTime;
        integralRight = Math.max(-maxIntegral, Math.min(maxIntegral, integralRight));
        double integralTermRight = kI * integralRight;
        
        double derivativeRight = (errorRight - lastErrorRight) / deltaTime;
        double derivativeTermRight = kD * derivativeRight;
        
        double feedforwardRight = kF * targetVelocity;
        
        double outputRight = (proportionalRight + integralTermRight + derivativeTermRight + feedforwardRight) * voltageCompensation;
        outputRight = Math.max(-1.0, Math.min(1.0, outputRight));

        // Set motor powers
        leftFlywheel.setPower(outputLeft);
        rightFlywheel.setPower(outputRight);

        // Update state
        lastErrorLeft = errorLeft;
        lastErrorRight = errorRight;
        lastTime = currentTime;

        // Check if ready (baseado na média dos erros)
        double avgError = (Math.abs(errorLeft) + Math.abs(errorRight)) / 2.0;
        if (avgError < READY_TOLERANCE) {
            if (!isReady) {
                readyTimer.reset();
            }
            isReady = readyTimer.seconds() >= READY_TIME;
        } else {
            isReady = false;
        }
    }

    /**
     * Update target velocity based on distance to target.
     * Usa ShooterDistanceToRPM com os pontos em ConstantsConf.Shooter (DISTANCE_LUT_POL, RPM_LUT).
     * A distância é limitada ao mínimo da LUT para evitar IllegalArgumentException (ex.: distância 0).
     */
    private void updateTargetVelocityFromDistance() {
        if (follower == null) return;

        Pose currentPose = follower.getPose();
        double dx = targetX - currentPose.getX();
        double dy = targetY - currentPose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);

        // Evita passar 0 ou valor inválido para a LUT (ex.: pose ainda não inicializada)
        if (distance < ConstantsConf.Shooter.DIST_NEAR_POL || !Double.isFinite(distance)) {
            distance = ConstantsConf.Shooter.DIST_NEAR_POL;
        }

        double targetRpm = distanceToRPM.getRPM(distance);
        double velocity = rpmToTicksPerSecond(targetRpm);
        if (targetVelocity == 0.0 || Math.abs(targetVelocity - velocity) > 10.0) {
            setTargetVelocity(velocity);
        }
    }

    /**
     * Stop the shooter.
     */
    public void stop() {
        setTargetVelocity(0.0);
        leftFlywheel.setPower(0.0);
        rightFlywheel.setPower(0.0);
        resetPID();
    }

    /**
     * Check if the shooter is at target velocity.
     *
     * @return true if shooter is ready to shoot
     */
    public boolean isReady() {
        return isReady && targetVelocity > 0;
    }

    /**
     * Get current velocity (média dos dois motores).
     *
     * @return Current velocity in ticks per second
     */
    public double getCurrentVelocity() {
        return currentVelocityAvg;
    }

    /**
     * Get target velocity.
     *
     * @return Target velocity in ticks per second
     */
    public double getTargetVelocity() {
        return targetVelocity;
    }

    /**
     * Get velocity error.
     *
     * @return Current error (target - current average)
     */
    public double getVelocityError() {
        return targetVelocity - currentVelocityAvg;
    }

    /**
     * Get distance to target.
     *
     * @return Distance in inches
     */
    public double getDistance() {
        if (follower == null) return 0.0;

        Pose currentPose = follower.getPose();
        double dx = targetX - currentPose.getX();
        double dy = targetY - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Get current battery voltage.
     *
     * @return Battery voltage
     */
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    /**
     * Get voltage compensation factor.
     *
     * @return Compensation factor
     */
    public double getVoltageCompensation() {
        return voltageCompensation;
    }

    /**
     * Reset PID state variables.
     */
    private void resetPID() {
        integralLeft = 0.0;
        lastErrorLeft = 0.0;
        integralRight = 0.0;
        lastErrorRight = 0.0;
        lastTime = System.nanoTime() / 1e9;
        isReady = false;
    }

    /**
     * Set PID coefficients (for tuning).
     *
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kF Feedforward gain
     */
    public void setPID(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        resetPID();
    }

    // Getters para telemetria individual dos motores
    public double getCurrentVelocityLeft() {
        return currentVelocityLeft;
    }

    public double getCurrentVelocityRight() {
        return currentVelocityRight;
    }
}
