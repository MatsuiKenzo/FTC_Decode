package org.firstinspires.ftc.teamcode.drive.national.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Turret nacional com DOIS servos contínuos que giram JUNTOS.
 * Os dois recebem a mesma potência: um sentido com power &gt; 0, outro com power &lt; 0 (0 = parado).
 * Ambos recebem a mesma potência (-1 a 1, 0 = parado).
 * Use TurretCalibrator para calibrar degreesPerSecondPerPower (relação engrenagem).
 */
public class NacionalTurret {
    private Follower follower;
    private CRServo leftServo;
    private CRServo rightServo;

    private double kP = 0.06;
    private double kI = 0.0;
    private double kD = 0.0005;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private ElapsedTime timer = new ElapsedTime();

    private double minLimit = -60.0;
    private double maxLimit = 260.0;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private boolean isLocked = false;
    private double lockedAngle = 0.0;
    private double currentAngle = 0.0;

    /** Graus por segundo por unidade de potência (relação engrenagem). Calibrável via TurretCalibrator. */
    private double degreesPerSecondPerPower = ConstantsConf.Nacional.TURRET_DEGREES_PER_SECOND_PER_POWER;

    /**
     * Inicializa a turret com dois servos contínuos (os dois giram juntos).
     *
     * @param leftServoName  Nome do CRServo esquerdo
     * @param rightServoName Nome do CRServo direito
     */
    public void init(HardwareMap hardwareMap, Follower follower, String leftServoName, String rightServoName) {
        this.follower = follower;
        leftServo = hardwareMap.get(CRServo.class, leftServoName);
        rightServo = hardwareMap.get(CRServo.class, rightServoName);
        degreesPerSecondPerPower = ConstantsConf.Nacional.TURRET_DEGREES_PER_SECOND_PER_POWER;
        stopRotation();
        timer.reset();
        currentAngle = 0.0;
    }

    public void setTargetPosition(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    public void lockAngle(double angle) {
        this.lockedAngle = angle;
        this.isLocked = true;
    }

    public void unlockAngle() {
        this.isLocked = false;
    }

    public void setLimits(double min, double max) {
        this.minLimit = min;
        this.maxLimit = max;
    }

    public void update() {
        double targetDegrees;

        if (isLocked) {
            targetDegrees = lockedAngle;
        } else {
            if (follower == null) {
                stopRotation();
                return;
            }
            Pose currentPose = follower.getPose();
            double dx = targetX - currentPose.getX();
            double dy = targetY - currentPose.getY();
            double absoluteAngleToTarget = Math.toDegrees(Math.atan2(dy, dx));
            double robotHeading = Math.toDegrees(currentPose.getHeading());
            double relativeTargetAngle = absoluteAngleToTarget - robotHeading;
            double error = relativeTargetAngle - currentAngle;
            while (error > 180) error -= 360;
            while (error < -180) error += 360;
            targetDegrees = currentAngle + error;
        }

        double constrainedTargetDegrees = Range.clip(targetDegrees, minLimit, maxLimit);
        double power = calculatePID(constrainedTargetDegrees, currentAngle);
        setRotationPower(power);

        double deltaTime = timer.seconds();
        if (deltaTime > 0 && deltaTime < 1.0) {
            currentAngle += power * degreesPerSecondPerPower * deltaTime;
            timer.reset();
        }
    }

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

    /** Controle manual: power -1 a 1 (0 = parado). Os dois servos giram juntos. */
    public void setManualPower(double power) {
        setRotationPower(power);
    }

    /**
     * Aplica potência e atualiza a estimativa de ângulo (para calibrator / teste manual).
     * Chame no loop com dt = tempo desde a última chamada.
     */
    public void setManualPowerAndIntegrate(double power, double dtSeconds) {
        setRotationPower(power);
        if (dtSeconds > 0 && dtSeconds < 1.0) {
            currentAngle += power * degreesPerSecondPerPower * dtSeconds;
        }
    }

    /** Os dois servos recebem a mesma potência. */
    private void setRotationPower(double power) {
        if (leftServo == null || rightServo == null) return;
        power = Range.clip(power, -1.0, 1.0);
        leftServo.setPower(power);
        rightServo.setPower(power);
    }

    public void stopRotation() {
        if (leftServo != null) leftServo.setPower(0.0);
        if (rightServo != null) rightServo.setPower(0.0);
    }

    public double getMotorAngle() {
        return currentAngle;
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void resetAngle(double angle) {
        currentAngle = angle;
        integralSum = 0.0;
        lastError = 0.0;
        timer.reset();
    }

    /** Define a relação graus/(s·potência). Use valor calibrado no TurretCalibrator. */
    public void setDegreesPerSecondPerPower(double value) {
        this.degreesPerSecondPerPower = value;
    }

    public double getDegreesPerSecondPerPower() {
        return degreesPerSecondPerPower;
    }
}
