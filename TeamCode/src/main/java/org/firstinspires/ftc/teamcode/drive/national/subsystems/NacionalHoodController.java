package org.firstinspires.ftc.teamcode.drive.national.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Controlador do hood/tilt do shooter nacional.
 * Ajusta a posição do servo (ângulo do hood) em função da distância até o alvo.
 *
 * IMPORTANTE: Esta classe pode ser DESATIVADA se o servo Tauron não estiver conectado.
 *
 * Para DESATIVAR:
 * 1. No RobotHardware, defina TILT_ENABLED = false antes de inicializar
 * 2. Ou não configure o servo "hood" no Robot Configuration
 *
 * Se o servo não estiver conectado e você tentar inicializar, o código vai capturar
 * a exceção e desativar automaticamente (tiltServo ficará null).
 *
 * Quando DESATIVADO:
 * - Todos os métodos são seguros (não fazem nada se tiltServo == null)
 * - update() não causa erros
 * - getCurrentAngle() retorna 0.0
 */
public class NacionalHoodController {
    private Follower follower;
    private Servo tiltServo;

    // Flag para ativar/desativar o tilt
    // Se false, todos os métodos são no-ops seguros
    private boolean tiltEnabled = false;

    // Coordenadas do alvo no campo
    private double targetX = 10.0;
    private double targetY = 100.0;

    // Limites de posição do servo (0.0–1.0) – AJUSTE NO ROBÔ
    private double minAngle = 0.40; // posição para tiro mais perto (hood mais alto ou mais baixo, depende da mecânica)
    private double maxAngle = 0.90; // posição para tiro mais longe

    // Distâncias onde cada ângulo vale
    private double minDistance = 20.0;   // quando estiver mais perto que isso, usa minAngle
    private double maxDistance = 120.0;  // quando estiver mais longe que isso, usa maxAngle

    /**
     * Inicializa o controlador do hood.
     *
     * @param hardwareMap hardwareMap do OpMode
     * @param follower    Follower do PedroPathing (para obter a pose atual)
     * @param servoName   nome do servo na configuração (ex.: "hood")
     * @return true se o servo foi encontrado e inicializado, false caso contrário
     */
    public boolean init(HardwareMap hardwareMap, Follower follower, String servoName) {
        this.follower = follower;

        try {
            tiltServo = hardwareMap.get(Servo.class, servoName);
            tiltServo.setPosition(minAngle); // Posição inicial segura
            tiltEnabled = true;
            return true;
        } catch (Exception e) {
            // Servo não encontrado ou não conectado - desativa silenciosamente
            tiltServo = null;
            tiltEnabled = false;
            return false;
        }
    }

    /**
     * Desativa o tilt manualmente (útil se você souber que não tem o servo conectado).
     */
    public void disable() {
        tiltEnabled = false;
        tiltServo = null;
    }

    /**
     * Verifica se o tilt está habilitado e funcionando.
     *
     * @return true se o tilt está ativo
     */
    public boolean isEnabled() {
        return tiltEnabled && tiltServo != null;
    }

    /** Define posição do alvo (cesta/goal) no campo. */
    public void setTargetPosition(double x, double y) {
        this.targetX = x;
        this.targetY = y;
    }

    /** Define faixa de distâncias usada na interpolação. */
    public void setDistanceBounds(double minD, double maxD) {
        this.minDistance = minD;
        this.maxDistance = maxD;
    }

    /** Define faixa de ângulos (posições de servo) usada na interpolação. */
    public void setAngleBounds(double minA, double maxA) {
        this.minAngle = minA;
        this.maxAngle = maxA;
    }

    /**
     * Atualiza a posição do hood de acordo com a distância atual.
     * Chame isso a cada loop do TeleOp/Auto.
     *
     * SEGURO: Não faz nada se o tilt estiver desativado.
     */
    public void update() {
        if (!tiltEnabled || tiltServo == null || follower == null) return;

        Pose pose = follower.getPose();

        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);

        double angle;
        if (distance <= minDistance) {
            angle = minAngle;
        } else if (distance >= maxDistance) {
            angle = maxAngle;
        } else {
            double t = (distance - minDistance) / (maxDistance - minDistance);
            angle = minAngle + t * (maxAngle - minAngle);
        }

        angle = Range.clip(angle, 0.0, 1.0);
        tiltServo.setPosition(angle);
    }

    /** Distância atual até o alvo. */
    public double getDistance() {
        if (follower == null) return 0.0;
        Pose pose = follower.getPose();
        double dx = targetX - pose.getX();
        double dy = targetY - pose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    /** Posição atual do servo (0.0–1.0). Retorna 0.0 se desativado. */
    public double getCurrentAngle() {
        return tiltServo != null ? tiltServo.getPosition() : 0.0;
    }

    /** Coloca o hood em uma posição "segura" (por exemplo, minAngle). */
    public void safePosition() {
        if (tiltEnabled && tiltServo != null) {
            tiltServo.setPosition(Range.clip(minAngle, 0.0, 1.0));
        }
    }
}
