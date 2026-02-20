package org.firstinspires.ftc.teamcode.drive.national.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Nova turret nacional controlada por DOIS servos contínuos (base giratória)
 * e integrada com PedroPathing para apontar automaticamente para alvos.
 * 
 * Features:
 * - Controle PID para posicionamento preciso da base giratória
 * - Integração com PedroPathing para cálculo de ângulo ao alvo
 * - Limites de zona configuráveis
 * - Modo lock para manter ângulo fixo
 * - Dois servos contínuos: quando um gira, o outro fica em "float" (power 0)
 * 
 * IMPORTANTE: O tilt/hood está implementado mas pode ser desativado se o servo não estiver conectado.
 * Veja o método init() e a constante TILT_ENABLED.
 */
public class NacionalTurret {
    private Follower follower;
    
    // Servos contínuos para girar a base
    private CRServo leftServo;   // gira para um lado
    private CRServo rightServo;  // gira para o outro lado

    // PID coefficients para controle da base
    private double kP = 0.06;
    private double kI = 0.0;
    private double kD = 0.0005;

    // PID state
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private ElapsedTime timer = new ElapsedTime();

    // Limits (em graus relativos ao robô)
    private double minLimit = -60.0;
    private double maxLimit = 260.0;

    // Target position
    private double targetX = 0.0;
    private double targetY = 0.0;

    // Lock mode
    private boolean isLocked = false;
    private double lockedAngle = 0.0;

    // Estado atual estimado da base (sem encoder, estimamos baseado no tempo e potência)
    // Em graus relativos ao robô
    private double currentAngle = 0.0;
    
    // Constante para estimar movimento angular baseado em potência e tempo
    // Ajuste este valor baseado nos testes físicos do robô
    private static final double DEGREES_PER_SECOND_PER_POWER = 30.0; // graus/s por unidade de power

    /**
     * Inicializa a turret nacional.
     *
     * @param hardwareMap HardwareMap from OpMode
     * @param follower PedroPathing Follower instance
     * @param leftServoName Name of the left CRServo in hardware map
     * @param rightServoName Name of the right CRServo in hardware map
     */
    public void init(HardwareMap hardwareMap, Follower follower, String leftServoName, String rightServoName) {
        this.follower = follower;
        
        leftServo = hardwareMap.get(CRServo.class, leftServoName);
        rightServo = hardwareMap.get(CRServo.class, rightServoName);

        // Para a base inicialmente
        stopRotation();
        
        timer.reset();
        currentAngle = 0.0;
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
     * @param angle Angle in degrees (relativo ao robô)
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
        double targetDegrees;

        if (isLocked) {
            // If locked, target is the locked angle
            targetDegrees = lockedAngle;
        } else {
            // Calculate angle to target using PedroPathing
            if (follower == null) {
                stopRotation();
                return;
            }
            
            Pose currentPose = follower.getPose();

            // Calculate absolute angle to target
            double dx = targetX - currentPose.getX();
            double dy = targetY - currentPose.getY();
            double absoluteAngleToTarget = Math.toDegrees(Math.atan2(dy, dx));

            // Calculate relative angle to robot
            double robotHeading = Math.toDegrees(currentPose.getHeading());
            double relativeTargetAngle = absoluteAngleToTarget - robotHeading;

            // Normalize error to [-180, 180] for shortest path
            double error = relativeTargetAngle - currentAngle;
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Target is current position + shortest path error
            targetDegrees = currentAngle + error;
        }

        // Apply limits
        double constrainedTargetDegrees = Range.clip(targetDegrees, minLimit, maxLimit);

        // PID control
        double power = calculatePID(constrainedTargetDegrees, currentAngle);
        
        // Aplica potência aos servos (um ativo, outro em float)
        setRotationPower(power);
        
        // Atualiza estimativa de ângulo baseado na potência aplicada
        double deltaTime = timer.seconds();
        if (deltaTime > 0 && deltaTime < 1.0) {
            currentAngle += power * DEGREES_PER_SECOND_PER_POWER * deltaTime;
            timer.reset();
        }
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

        if (deltaTime > 0 && deltaTime < 1.0) {
            integralSum += error * deltaTime;
            integralSum = Range.clip(integralSum, -0.5, 0.5);

            double derivative = (error - lastError) / deltaTime;
            lastError = error;

            double output = (kP * error) + (kI * integralSum) + (kD * derivative);
            return Range.clip(output, -1.0, 1.0);
        }
        return 0.0;
    }

    /**
     * Gira a turret com potência -1.0 a 1.0.
     *
     * Convenção:
     * - power > 0: gira usando servo esquerdo, direito em float (power 0)
     * - power < 0: gira usando servo direito, esquerdo em float (power 0)
     * - power = 0: ambos em float (parados, sem segurar posição)
     */
    private void setRotationPower(double power) {
        if (leftServo == null || rightServo == null) return;

        power = Range.clip(power, -1.0, 1.0);

        if (power > 0.0) {
            // Gira usando servo esquerdo, direito em float
            leftServo.setPower(power);
            rightServo.setPower(0.0);
        } else if (power < 0.0) {
            // Gira usando servo direito, esquerdo em float
            leftServo.setPower(0.0);
            rightServo.setPower(-power); // inverte para manter convenção
        } else {
            // Ambos em float
            stopRotation();
        }
    }

    /** Para a base giratória (ambos em power 0). */
    public void stopRotation() {
        if (leftServo != null) {
            leftServo.setPower(0.0);
        }
        if (rightServo != null) {
            rightServo.setPower(0.0);
        }
    }

    /**
     * Get current motor angle in degrees (estimado).
     *
     * @return Current angle in degrees (relativo ao robô)
     */
    public double getMotorAngle() {
        return currentAngle;
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

    /**
     * Reseta a estimativa de ângulo (útil quando você sabe a posição real).
     */
    public void resetAngle(double angle) {
        currentAngle = angle;
        integralSum = 0.0;
        lastError = 0.0;
        timer.reset();
    }
}
