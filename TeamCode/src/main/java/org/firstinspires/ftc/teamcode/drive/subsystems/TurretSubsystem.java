package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Turret subsystem integrado com PedroPathing.
 *
 * Controla a base giratória (turret) usando PID para apontar para alvos no campo.
 * Configurado para motor HD Hex (3:1) + Engrenagem externa (5:1) = 15:1 total.
 *
 * Features:
 * - Controle PID para posicionamento preciso
 * - Integração com PedroPathing para cálculo de ângulo ao alvo
 * - Limites de zona configuráveis
 * - Modo lock para manter ângulo fixo
 */
public class TurretSubsystem {
    private Follower follower;
    private DcMotorEx motor;

    // PID coefficients
    private double kP = 0.06;
    private double kI = 0.0;
    private double kD = 0.0005;

    // PID state
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private ElapsedTime timer = new ElapsedTime();

    // Limits
    private double minLimit = -60.0;
    private double maxLimit = 260.0;

    // Target position
    private double targetX = 0.0;
    private double targetY = 0.0;

    // Lock mode
    private boolean isLocked = false;
    private double lockedAngle = 0.0;

    // Motor configuration
    // Redução Total 15:1 (28 ticks * 3 * 5 = 420 ticks por revolução do motor)
    // Mas o encoder do HD Hex tem 28 ticks, então: 28 * 3 * 5 = 420 ticks por revolução do output
    private static final double TICKS_PER_REV = 2100.0; // Ajuste conforme sua configuração

    /**
     * Initialize the turret subsystem.
     *
     * @param hardwareMap HardwareMap from OpMode
     * @param follower PedroPathing Follower instance
     * @param motorName Name of the turret motor in hardware map
     */
    public void init(HardwareMap hardwareMap, Follower follower, String motorName) {
        this.follower = follower;
        this.motor = hardwareMap.get(DcMotorEx.class, motorName);

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        timer.reset();
    }

    /**
     * Set the target position for aiming.
     *
     * @param x Target X coordinate (inches)
     * @param y Target Y coordinate (inches)
     */
    public void setTargetPosition(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    /**
     * Lock the turret to a specific angle.
     *
     * @param angle Angle in degrees
     */
    public void lockAngle(double angle) {
        this.lockedAngle = angle;
        this.isLocked = true;
    }

    /**
     * Unlock the turret, returning to automatic target tracking.
     */
    public void unlockAngle() {
        this.isLocked = false;
    }

    /**
     * Set angle limits.
     *
     * @param min Minimum angle in degrees
     * @param max Maximum angle in degrees
     */
    public void setLimits(double min, double max) {
        this.minLimit = min;
        this.maxLimit = max;
    }

    /**
     * Update the turret control loop.
     * Call this in your OpMode's loop() method.
     */
    public void update() {
        double currentMotorAngle = getMotorAngle();
        double targetDegrees;

        if (isLocked) {
            // If locked, target is the locked angle
            targetDegrees = lockedAngle;
        } else {
            // Calculate angle to target using PedroPathing
            Pose currentPose = follower.getPose();

            // Calculate absolute angle to target
            double dx = targetX - currentPose.getX();
            double dy = targetY - currentPose.getY();
            double absoluteAngleToTarget = Math.toDegrees(Math.atan2(dy, dx));

            // Calculate relative angle to robot
            double robotHeading = Math.toDegrees(currentPose.getHeading());
            double relativeTargetAngle = absoluteAngleToTarget - robotHeading;

            // Normalize error to [-180, 180] for shortest path
            double error = relativeTargetAngle - (currentMotorAngle % 360);
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Target is current position + shortest path error
            targetDegrees = currentMotorAngle + error;
        }

        // Apply limits
        double constrainedTargetDegrees = Range.clip(targetDegrees, minLimit, maxLimit);

        // PID control
        double power = calculatePID(constrainedTargetDegrees, currentMotorAngle);
        motor.setPower(power);
    }

    /**
     * Calculate PID output.
     *
     * @param target Target angle in degrees
     * @param current Current angle in degrees
     * @return Motor power [-1.0, 1.0]
     */
    private double calculatePID(double target, double current) {
        double error = target - current;
        double deltaTime = timer.seconds();

        if (deltaTime > 0) {
            integralSum += error * deltaTime;
            integralSum = Range.clip(integralSum, -0.5, 0.5);

            double derivative = (error - lastError) / deltaTime;
            lastError = error;
            timer.reset();

            double output = (kP * error) + (kI * integralSum) + (kD * derivative);
            return Range.clip(output, -1.0, 1.0);
        }
        return 0.0;
    }

    /**
     * Get current motor angle in degrees.
     *
     * @return Current angle in degrees
     */
    public double getMotorAngle() {
        return (motor.getCurrentPosition() / TICKS_PER_REV) * 360.0;
    }

    /**
     * Set PID coefficients.
     *
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
}
