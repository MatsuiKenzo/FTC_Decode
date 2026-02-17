package org.firstinspires.ftc.teamcode.drive.actuators;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Localizador que funde odometria Pinpoint com visão Limelight3A usando média ponderada
 * (filtro de Kalman simplificado).
 * Usa APENAS AprilTags 20 e 24 para localização.
 * Pesos adaptativos: mais tags visíveis = maior confiança na visão; posição afeta o peso.
 * Posições do servo: 0 = posição padrão; 1 = alinhado com o shooter.
 */
public class KalmanFilterLocalizer {
    private static final int TAG_BLUE_GOAL = 20;
    private static final int TAG_RED_GOAL = 24;

    private Follower follower;
    private Limelight3A limelight;

    private Pose fusedPose = new Pose(0, 0, 0);

    /** Desvio padrão do Pinpoint (odometria) - menor = mais confiança */
    private double pinpointStdDev = 0.05;
    /** Desvio base da visão - ajustado por quantidade de tags e distância */
    private double baseVisionStdDev = 0.15;

    /** Máxima variação de heading por loop (rad) para aceitar visão — evita blur em giros rápidos */
    private double maxHeadingChangeRadPerLoop = 0.25;

    private Pose lastLimelightPose = new Pose(0, 0, 0);
    private int lastValidTagCount = 0;
    private boolean hasTarget = false;
    private double lastHeadingRad = 0;

    /** Se true, aplica fusão Pinpoint + Limelight. Se false, usa só Pinpoint. */
    private boolean visionFusionEnabled = true;

    /**
     * Inicializa o localizador. Retorna true se a Limelight foi encontrada e inicializada.
     */
    public boolean init(HardwareMap hardwareMap, Follower follower) {
        this.follower = follower;
        this.fusedPose = follower.getPose();
        this.lastHeadingRad = fusedPose.getHeading();
        try {
            this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
            this.limelight.pipelineSwitch(0);
            this.limelight.start();
            return true;
        } catch (Exception e) {
            this.limelight = null;
            return false;
        }
    }

    /**
     * Deve ser chamado no loop inteiro, APÓS follower.update().
     * Não faz nada se a Limelight não foi inicializada.
     */
    public void update() {
        if (limelight == null) {
            return;
        }
        if (!visionFusionEnabled) {
            lastHeadingRad = follower.getPose().getHeading();
            return;
        }
        Pose currentPinpointPose = follower.getPose();
        double currentHeading = currentPinpointPose.getHeading();
        double currentHeadingDeg = Math.toDegrees(currentHeading);

        // MegaTag2: enviar orientação para a Limelight
        limelight.updateRobotOrientation(currentHeadingDeg);

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            hasTarget = false;
            fusedPose = currentPinpointPose;
            lastHeadingRad = currentHeading; // sempre atualizar para cálculo de omega
            return;
        }

        // Filtrar: aceitar apenas tags 20 e 24 (localização de arena)
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        int validTagCount = 0;
        boolean hasInvalidTag = false;
        for (LLResultTypes.FiducialResult fr : fiducials) {
            int id = fr.getFiducialId();
            if (id == TAG_BLUE_GOAL || id == TAG_RED_GOAL) {
                validTagCount++;
            } else {
                hasInvalidTag = true;
            }
        }

        if (hasInvalidTag || validTagCount == 0) {
            hasTarget = false;
            fusedPose = currentPinpointPose;
            lastHeadingRad = currentHeading;
            return;
        }

        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            hasTarget = false;
            fusedPose = currentPinpointPose;
            lastHeadingRad = currentHeading;
            return;
        }

        // Filtrar por variação de heading (visão tende a borrar em giros rápidos)
        double headingChange = Math.abs(normalizeAngle(currentHeading - lastHeadingRad));
        if (headingChange > maxHeadingChangeRadPerLoop) {
            fusedPose = currentPinpointPose;
            lastHeadingRad = currentHeading;
            return;
        }
        lastHeadingRad = currentHeading;

        // Conversão: metros -> polegadas; yaw em graus -> radianos
        double visionX = botpose.getPosition().x * 39.3701;
        double visionY = botpose.getPosition().y * 39.3701;
        double visionHeading = Math.toRadians(botpose.getOrientation().getYaw());

        lastLimelightPose = new Pose(visionX, visionY, visionHeading);
        lastValidTagCount = validTagCount;
        hasTarget = true;

        // Desvio padrão da visão: mais tags = menor desvio = maior confiança
        double currentVisionStdDev = baseVisionStdDev * (1.0 / validTagCount);

        // Peso posicional: quando mais perto de uma tag, visão é mais confiável
        double distToVision = Math.hypot(
                visionX - currentPinpointPose.getX(),
                visionY - currentPinpointPose.getY()
        );
        if (distToVision > 24) {
            currentVisionStdDev *= 1.5; // Desconfiar se pose da visão diverge muito do pinpoint
        }

        // Média ponderada (inverso da variância = 1/stdDev²)
        double wPinpoint = 1.0 / (pinpointStdDev * pinpointStdDev);
        double wVision = 1.0 / (currentVisionStdDev * currentVisionStdDev);
        double totalWeight = wPinpoint + wVision;

        double fusedX = (currentPinpointPose.getX() * wPinpoint + visionX * wVision) / totalWeight;
        double fusedY = (currentPinpointPose.getY() * wPinpoint + visionY * wVision) / totalWeight;

        double angleDiff = normalizeAngle(visionHeading - currentPinpointPose.getHeading());
        double fusedHeading = currentPinpointPose.getHeading() + (angleDiff * (wVision / totalWeight));

        fusedPose = new Pose(fusedX, fusedY, fusedHeading);
        follower.setPose(fusedPose);
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    public Pose getFusedPose() {
        return fusedPose;
    }

    public Pose getLimelightPose() {
        return lastLimelightPose;
    }

    public int getValidTagCount() {
        return lastValidTagCount;
    }

    public boolean hasVision() {
        return hasTarget;
    }

    public void setPinpointStdDev(double pinpointStdDev) {
        this.pinpointStdDev = pinpointStdDev;
    }

    public void setBaseVisionStdDev(double baseVisionStdDev) {
        this.baseVisionStdDev = baseVisionStdDev;
    }

    /** Liga/desliga a fusão com a Limelight em tempo real. */
    public void setVisionFusionEnabled(boolean enabled) {
        this.visionFusionEnabled = enabled;
    }

    public boolean isVisionFusionEnabled() {
        return visionFusionEnabled;
    }
}
