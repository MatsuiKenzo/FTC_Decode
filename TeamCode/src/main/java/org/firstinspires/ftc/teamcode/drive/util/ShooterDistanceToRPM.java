package org.firstinspires.ftc.teamcode.drive.util;

import com.seattlesolvers.solverslib.util.InterpLUT;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Encapsula a curva distância -> RPM do shooter usando InterpLUT (SolversLib/FTCLib).
 * Permite calibração com vários pontos; a interpolação entre eles é feita pela própria LUT.
 */
public class ShooterDistanceToRPM {

    private InterpLUT lut;
    private boolean built = false;
    private double minDistance = 0;
    private double maxDistance = 0;

    /**
     * Constrói a LUT a partir dos pontos definidos em ConstantsConf.Shooter.
     */
    public void buildFromConstants() {
        double[] distances = ConstantsConf.Shooter.DISTANCE_LUT_POL;
        double[] rpms = ConstantsConf.Shooter.RPM_LUT;

        if (distances == null || rpms == null || distances.length != rpms.length || distances.length == 0) {
            built = false;
            return;
        }

        lut = new InterpLUT();
        minDistance = distances[0];
        maxDistance = distances[0];

        for (int i = 0; i < distances.length; i++) {
            lut.add(distances[i], rpms[i]);
            if (distances[i] < minDistance) minDistance = distances[i];
            if (distances[i] > maxDistance) maxDistance = distances[i];
        }

        lut.createLUT();
        built = true;
    }

    /**
     * Retorna o RPM interpolado para a distância dada (em polegadas).
     * Adicionada proteção para evitar IllegalArgumentException da SolversLib.
     */
    public double getRPM(double distanceInches) {
        if (!built) {
            buildFromConstants();
        }

        if (!built) {
            return ConstantsConf.Shooter.RPM_NEAR; // fallback
        }

        // PROTEÇÃO: Clamping da distância para os limites da LUT
        // Isso evita que a SolversLib lance erro se o robô estiver muito perto ou longe.
        double clampedDistance = Math.max(minDistance, Math.min(maxDistance, distanceInches));

        return lut.get(clampedDistance);
    }

    public void invalidate() {
        built = false;
    }
}
