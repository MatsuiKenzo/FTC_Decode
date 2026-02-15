package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Shooter subsystem integrado com PedroPathing usando velocity control com PID.
 *
 * Features:
 * - Velocity PID control para velocidade consistente do flywheel
 * - Compensação automática de bateria
 * - Ajuste automático de velocidade baseado na distância ao alvo (usando PedroPathing)
 * - Feedforward para resposta rápida
 * - Estado "ready" para garantir tiros precisos
 *
 * Integrado com PedroPathing para calcular distância ao alvo e ajustar velocidade automaticamente.
 */
public class PedroPathingShooter {
    private Follower follower;
    private DcMotorEx flywheelMotor;
    private VoltageSensor voltageSensor;

    // Controle de velocidade
    private double targetVelocity = 0.0;
    private double currentVelocity = 0.0;
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

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;

    // Battery compensation (ConstantsConf.Shooter.NOMINAL_VOLTAGE: abaixe se com bateria cheia o tiro ficar forte)
    private double voltageCompensation = 1.0;

    // Ready detection
    private ElapsedTime readyTimer = new ElapsedTime();
    private static final double READY_TOLERANCE = 50.0; // RPM tolerance
    private static final double READY_TIME = 0.2; // seconds

    /** Se true, update() recalcula target por distância ao alvo; se false, só usa o RPM/velocidade definida manualmente (ex: Shooter Tuner). */
    private boolean useDistanceBasedVelocity = true;

    /**
     * Initialize the shooter subsystem with PedroPathing integration.
     *
     * @param hardwareMap HardwareMap from OpMode
     * @param follower PedroPathing Follower instance
     * @param motorName Name of the flywheel motor in hardware map
     */
    public PedroPathingShooter(HardwareMap hardwareMap, Follower follower, String motorName) {
        this.follower = follower;
        this.flywheelMotor = hardwareMap.get(DcMotorEx.class, motorName);

        // Configure motor for velocity control
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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
     * @param velocity Target velocity in ticks per second
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
     * Em false, o shooter obedece apenas ao RPM/velocidade definido por setTargetRPM/setTargetVelocity (ex: Shooter Tuner).
     */
    public void setUseDistanceBasedVelocity(boolean use) {
        this.useDistanceBasedVelocity = use;
    }

    /**
     * Get current speed in RPM.
     *
     * @return Current velocity in revolutions per minute
     */
    public double getCurrentRPM() {
        return ticksPerSecondToRPM(currentVelocity);
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

        // Get current velocity
        currentVelocity = flywheelMotor.getVelocity();

        // Calculate voltage compensation
        double currentVoltage = voltageSensor.getVoltage();
        voltageCompensation = ConstantsConf.Shooter.NOMINAL_VOLTAGE / currentVoltage;

        // Calculate error
        double error = targetVelocity - currentVelocity;

        // Calculate time delta
        double currentTime = System.nanoTime() / 1e9;
        double deltaTime = currentTime - lastTime;

        if (deltaTime == 0 || deltaTime > 1.0) {
            deltaTime = 0.02; // Default to 20ms
        }

        // Proportional term
        double proportional = kP * error;

        // Integral term (with anti-windup)
        integral += error * deltaTime;
        double maxIntegral = 0.5 / (kI > 0 ? kI : 1.0);
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        double integralTerm = kI * integral;

        // Derivative term
        double derivative = (error - lastError) / deltaTime;
        double derivativeTerm = kD * derivative;

        // Feedforward term
        double feedforward = kF * targetVelocity;

        // Calculate output power
        double output = (proportional + integralTerm + derivativeTerm + feedforward) * voltageCompensation;

        // Clamp output
        output = Math.max(-1.0, Math.min(1.0, output));

        // Set motor power
        flywheelMotor.setPower(output);

        // Update state
        lastError = error;
        lastTime = currentTime;

        // Check if ready
        if (Math.abs(error) < READY_TOLERANCE) {
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
     * Uses calibration points: perto (63.1 pol, 3062 RPM), meio (98.7 pol, 3686 RPM), longe (145.5 pol, 4250 RPM).
     */
    private void updateTargetVelocityFromDistance() {
        if (follower == null) return;

        Pose currentPose = follower.getPose();
        double dx = targetX - currentPose.getX();
        double dy = targetY - currentPose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);

        double dNear = ConstantsConf.Shooter.DIST_NEAR_POL;
        double dMid = ConstantsConf.Shooter.DIST_MID_POL;
        double dFar = ConstantsConf.Shooter.DIST_FAR_POL;
        double rpmNear = ConstantsConf.Shooter.RPM_NEAR;
        double rpmMid = ConstantsConf.Shooter.RPM_MID;
        double rpmFar = ConstantsConf.Shooter.RPM_FAR;

        double targetRpm;
        if (distance <= dNear) {
            targetRpm = rpmNear;
        } else if (distance < dMid) {
            targetRpm = rpmNear + (distance - dNear) * (rpmMid - rpmNear) / (dMid - dNear);
        } else if (distance < dFar) {
            targetRpm = rpmMid + (distance - dMid) * (rpmFar - rpmMid) / (dFar - dMid);
        } else {
            targetRpm = rpmFar;
        }

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
        flywheelMotor.setPower(0.0);
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
     * Get current velocity.
     *
     * @return Current velocity in ticks per second
     */
    public double getCurrentVelocity() {
        return currentVelocity;
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
     * @return Current error (target - current)
     */
    public double getVelocityError() {
        return targetVelocity - currentVelocity;
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
     * Get current power (for compatibility with old code).
     *
     * @return Current motor power
     */
    public double getCurrentPower() {
        return flywheelMotor.getPower();
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
        integral = 0.0;
        lastError = 0.0;
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
    }

    /**
     * Get the motor instance (for compatibility with old code).
     *
     * @return DcMotorEx instance
     */
    public DcMotorEx getMotor() {
        return flywheelMotor;
    }
}
