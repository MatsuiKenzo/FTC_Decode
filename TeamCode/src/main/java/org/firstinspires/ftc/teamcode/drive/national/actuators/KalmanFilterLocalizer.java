package org.firstinspires.ftc.teamcode.drive.national.actuators;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;


public class KalmanFilterLocalizer {
    private static final int TAG_BLUE_GOAL = 20;
    private static final int TAG_RED_GOAL = 24;

    /** Offset para converter Limelight (centro = 0,0) -> Pinpoint (canto = 0,0). Campo 144": centro = 72 pol. */
    private double limelightOffsetXInches = 72.0;
    private double limelightOffsetYInches = 72.0;

    private Follower follower;
    private Limelight3A limelight;

    private Pose fusedPose = new Pose(0, 0, 0);

    /** Desvio padrão do Pinpoint - menor = mais confiança */
    private double pinpointStdDev = 0.05;

    /** Desvio base da visão */
    private double baseVisionStdDev = 0.15;

    /** Máxima variação de heading por loop para aceitar visão desconsiderando blur em giros rápidos */
    private double maxHeadingChangeRadPerLoop = 0.25;

    private Pose lastLimelightPose = new Pose(0, 0, 0);
    private int lastValidTagCount = 0;
    private boolean hasTarget = false;
    private double lastHeadingRad = 0;

    /** Se true, aplica fusão Pinpoint + Limelight. Se false, usa só Pinpoint. */
    private boolean visionFusionEnabled = true;

    /** Se false, só funde X e Y com a visão; heading fica do Pinpoint. */
    private boolean fuseVisionHeading = false;

    /** Motivo da última rejeição da visão(telemetria) */
    private String lastRejectionReason = "";

    /** IDs das tags vistas no último resultado */
    private String lastFiducialIdsSeen = "";

    /** Última pose da LL em metros. */
    private double lastVisionXMeters = 0, lastVisionYMeters = 0, lastVisionYawDeg = 0;

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


    public void update() {
        if (limelight == null) {
            return;
        }
        if (!visionFusionEnabled) {
            lastHeadingRad = follower.getPose().getHeading();
            lastRejectionReason = "fusao OFF";
            return;
        }
        Pose currentPinpointPose = follower.getPose();
        double currentHeading = currentPinpointPose.getHeading();
        double currentHeadingDeg = Math.toDegrees(currentHeading);

        // MegaTag2: enviar orientação para a Limelight
        limelight.updateRobotOrientation(currentHeadingDeg);

        LLResult result = limelight.getLatestResult();
        if (result == null) {
            hasTarget = false;
            fusedPose = currentPinpointPose;
            lastHeadingRad = currentHeading;
            lastRejectionReason = "result null (LL sem dados?)";
            lastFiducialIdsSeen = "";
            return;
        }
        if (!result.isValid()) {
            hasTarget = false;
            fusedPose = currentPinpointPose;
            lastHeadingRad = currentHeading;
            lastRejectionReason = "result invalid";
            lastFiducialIdsSeen = "";
            return;
        }

        // Filtrar: aceitar apenas tags 20 e 24
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        StringBuilder idsSeen = new StringBuilder();
        int validTagCount = 0;
        boolean hasInvalidTag = false;
        for (LLResultTypes.FiducialResult fr : fiducials) {
            int id = fr.getFiducialId();
            if (idsSeen.length() > 0) idsSeen.append(", ");
            idsSeen.append(id);
            if (id == TAG_BLUE_GOAL || id == TAG_RED_GOAL) {
                validTagCount++;
            } else {
                hasInvalidTag = true;
            }
        }
        lastFiducialIdsSeen = idsSeen.length() > 0 ? idsSeen.toString() : "nenhuma";

        if (hasInvalidTag || validTagCount == 0) {
            hasTarget = false;
            fusedPose = currentPinpointPose;
            lastHeadingRad = currentHeading;
            lastRejectionReason = hasInvalidTag ? "tag obelisk (21/22/23) vista - ignorar" : "sem tag 20 ou 24";
            return;
        }

        Pose3D botpose = result.getBotpose();
        if (botpose == null) {
            hasTarget = false;
            fusedPose = currentPinpointPose;
            lastHeadingRad = currentHeading;
            lastRejectionReason = "botpose null";
            return;
        }

        // Filtrar por variação de heading (visão borrada)
        double headingChange = Math.abs(normalizeAngle(currentHeading - lastHeadingRad));
        if (headingChange > maxHeadingChangeRadPerLoop) {
            fusedPose = currentPinpointPose;
            lastHeadingRad = currentHeading;
            lastRejectionReason = "giro rapido (blur)";
            return;
        }
        lastRejectionReason = "";
        lastHeadingRad = currentHeading;

        // Valores brutos em metros (para calibração)
        lastVisionXMeters = botpose.getPosition().x;
        lastVisionYMeters = botpose.getPosition().y;
        lastVisionYawDeg = botpose.getOrientation().getYaw();

        // Conversão: Limelight (centro = 0,0, metros) -> mesmo sistema do Pinpoint (canto = 0,0, polegadas)
        double visionXInches = lastVisionXMeters * 39.3701 + limelightOffsetXInches;
        double visionYInches = lastVisionYMeters * 39.3701 + limelightOffsetYInches;
        double visionHeading = Math.toRadians(lastVisionYawDeg);

        lastLimelightPose = new Pose(visionXInches, visionYInches, visionHeading);
        lastValidTagCount = validTagCount;
        hasTarget = true;

        // Desvio padrão da visão: mais tags = menor desvio = maior confiança
        double currentVisionStdDev = baseVisionStdDev * (1.0 / validTagCount);

        // Peso posicional: quando mais perto de uma tag, visão é mais confiável
        double distToVision = Math.hypot(
                visionXInches - currentPinpointPose.getX(),
                visionYInches - currentPinpointPose.getY()
        );
        if (distToVision > 24) {
            currentVisionStdDev *= 1.5; // Desconfiar se pose da visão diverge muito do pinpoint
        }

        // Média ponderada (inverso da variância = 1/stdDev²)
        double wPinpoint = 1.0 / (pinpointStdDev * pinpointStdDev);
        double wVision = 1.0 / (currentVisionStdDev * currentVisionStdDev);
        double totalWeight = wPinpoint + wVision;

        double fusedX = (currentPinpointPose.getX() * wPinpoint + visionXInches * wVision) / totalWeight;
        double fusedY = (currentPinpointPose.getY() * wPinpoint + visionYInches * wVision) / totalWeight;

        double angleDiff = normalizeAngle(visionHeading - currentPinpointPose.getHeading());
        double fusedHeading = fuseVisionHeading
                ? (currentPinpointPose.getHeading() + (angleDiff * (wVision / totalWeight)))
                : currentPinpointPose.getHeading();

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

    /** Se true, funde também o heading com a visão; se false, só X/Y (menos tremidas na turret). */
    public void setFuseVisionHeading(boolean fuse) {
        this.fuseVisionHeading = fuse;
    }

    public boolean isVisionFusionEnabled() {
        return visionFusionEnabled;
    }

    /** Motivo da última rejeição (para diagnóstico). Vazio se visão foi aceita. */
    public String getLastRejectionReason() {
        return lastRejectionReason;
    }

    /** IDs das tags vistas no último frame. */
    public String getLastFiducialIdsSeen() {
        return lastFiducialIdsSeen;
    }

    /** Pose da LL em metros (X). Campo FTC: centro = (0,0), eixos = ±1.83 m. */
    public double getLastVisionXMeters() { return lastVisionXMeters; }
    public double getLastVisionYMeters() { return lastVisionYMeters; }
    public double getLastVisionYawDeg() { return lastVisionYawDeg; }

    /** Ajusta o offset Limelight -> Pinpoint (padrão 72, 72 para campo 144" com centro em 72). */
    public void setLimelightOffsetInches(double offsetX, double offsetY) {
        this.limelightOffsetXInches = offsetX;
        this.limelightOffsetYInches = offsetY;
    }
}
