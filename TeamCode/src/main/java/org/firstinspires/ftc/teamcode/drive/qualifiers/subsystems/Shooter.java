package org.firstinspires.ftc.teamcode.drive.qualifiers.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Shooter subsystem with velocity control using PID.
 *
 * Features:
 * - Velocity PID control for consistent flywheel speed
 * - Battery voltage compensation to maintain consistent power regardless of battery level
 * - Feedforward control for faster response
 * - Ready state detection
 *
 * This ensures consistent shot power regardless of battery voltage.
 */
public class Shooter {
    private DcMotorEx flywheelMotor;
    private VoltageSensor voltageSensor;

    private double targetVelocity = 0.0;
    private double currentVelocity = 0.0;
    private boolean isReady = false;

    private ElapsedTime readyTimer = new ElapsedTime();
    private static final double READY_TOLERANCE = 50.0; // RPM tolerance for "ready" state
    private static final double READY_TIME = 0.2; // seconds at target velocity to be considered ready

    // PID coefficients (tune these for your robot)
    private double kP = ConstantsConf.Shooter.KP;
    private double kI = ConstantsConf.Shooter.KI;
    private double kD = ConstantsConf.Shooter.KD;
    private double kF = ConstantsConf.Shooter.KF; // Feedforward

    // PID state variables
    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;

    // Battery compensation
    private static final double NOMINAL_VOLTAGE = 12.0; // Nominal battery voltage
    private double voltageCompensation = 1.0;

    /**
     * Initialize the shooter subsystem.
     *
     * @param hardwareMap HardwareMap from OpMode
     * @param motorName Name of the flywheel motor in hardware map
     */
    public Shooter(HardwareMap hardwareMap, String motorName) {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, motorName);

        // Configure motor for velocity control
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Get voltage sensor (usually from the expansion hub)
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Reset PID state
        resetPID();
    }

    /**
     * Set the target velocity for the flywheel.
     *
     * @param velocity Target velocity in ticks per second
     */
    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
        isReady = false;
        readyTimer.reset();
    }

    /**
     * Update the shooter PID control loop.
     * Call this in your OpMode's loop() method.
     */
    public void update() {
        // Get current velocity
        currentVelocity = flywheelMotor.getVelocity();

        // Calculate voltage compensation
        double currentVoltage = voltageSensor.getVoltage();
        voltageCompensation = NOMINAL_VOLTAGE / currentVoltage;

        // Calculate error
        double error = targetVelocity - currentVelocity;

        // Calculate time delta
        double currentTime = System.nanoTime() / 1e9;
        double deltaTime = currentTime - lastTime;

        if (deltaTime == 0 || deltaTime > 1.0) {
            deltaTime = 0.02; // Default to 20ms if time is invalid
        }

        // Proportional term
        double proportional = kP * error;

        // Integral term (with anti-windup)
        integral += error * deltaTime;
        // Limit integral to prevent windup
        double maxIntegral = 0.5 / kI; // Prevent integral from exceeding 50% of max power
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        double integralTerm = kI * integral;

        // Derivative term
        double derivative = (error - lastError) / deltaTime;
        double derivativeTerm = kD * derivative;

        // Feedforward term (helps reach target faster)
        double feedforward = kF * targetVelocity;

        // Calculate output power
        double output = (proportional + integralTerm + derivativeTerm + feedforward) * voltageCompensation;

        // Clamp output to valid range [-1, 1]
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
     * @return Compensation factor (1.0 = nominal voltage)
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
}
