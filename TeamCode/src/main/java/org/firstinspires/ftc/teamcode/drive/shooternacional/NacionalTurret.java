package org.firstinspires.ftc.teamcode.drive.shooternacional;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Nova turret nacional controlada por DOIS servos contínuos (base giratória)
 * e UM servo de ângulo (tilt/elevação).
 *
 * NÃO está ligada ao robô atual (nenhum TeleOp/Autônomo usa esta classe ainda).
 * Use apenas em testes separados.
 */
public class NacionalTurret {

    // Servos contínuos para girar a base
    private CRServo leftServo;   // gira para um lado
    private CRServo rightServo;  // gira para o outro lado

    // Servo de ângulo da arma (tilt)
    private Servo tiltServo;

    // Limites de posição do tilt (0.0–1.0) — ajuste depois no robô
    private double minTilt = 0.0;
    private double maxTilt = 1.0;

    // Posição atual de tilt (para conveniência)
    private double currentTilt = 0.5;

    /**
     * Inicializa a turret nacional.
     *
     * @param hardwareMap hardware map do OpMode
     * @param leftName    nome do CRServo esquerdo (ex.: \"turret_left\")
     * @param rightName   nome do CRServo direito (ex.: \"turret_right\")
     * @param tiltName    nome do servo de tilt (ex.: \"turret_tilt\")
     */
    public void init(HardwareMap hardwareMap, String leftName, String rightName, String tiltName) {
        leftServo = hardwareMap.get(CRServo.class, leftName);
        rightServo = hardwareMap.get(CRServo.class, rightName);
        tiltServo = hardwareMap.get(Servo.class, tiltName);

        // Posição inicial segura de tilt
        currentTilt = 0.5;
        tiltServo.setPosition(currentTilt);

        // Para a base inicialmente
        stopRotation();
    }

    /**
     * Define limites do ângulo do tilt (em posição de servo 0.0–1.0).
     * Útil para evitar bater em batentes mecânicos.
     */
    public void setTiltLimits(double minTilt, double maxTilt) {
        this.minTilt = minTilt;
        this.maxTilt = maxTilt;
        currentTilt = clamp(currentTilt, minTilt, maxTilt);
        if (tiltServo != null) {
            tiltServo.setPosition(currentTilt);
        }
    }

    /**
     * Gira a turret com potência -1.0 a 1.0.
     *
     * Convenção:
     * - power > 0: gira para um lado (left ativo, right em float)
     * - power < 0: gira para o outro lado (right ativo, left em float)
     * - power = 0: ambos em float (parados, sem segurar posição)
     */
    public void setRotationPower(double power) {
        if (leftServo == null || rightServo == null) return;

        power = clamp(power, -1.0, 1.0);

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
     * Define a posição absoluta do tilt (0.0–1.0, respeitando limites).
     */
    public void setTiltPosition(double position) {
        if (tiltServo == null) return;
        currentTilt = clamp(position, minTilt, maxTilt);
        tiltServo.setPosition(currentTilt);
    }

    /**
     * Incrementa o tilt em delta (positivo = sobe, negativo = desce).
     */
    public void changeTilt(double delta) {
        setTiltPosition(currentTilt + delta);
    }

    public double getTiltPosition() {
        return currentTilt;
    }

    // Helpers
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}

