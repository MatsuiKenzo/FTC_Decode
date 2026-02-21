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
     * Proteção para evitar IllegalArgumentException da SolversLib (valor fora dos limites da LUT).
     */
    public double getRPM(double distanceInches) {
        if (!built) {
            buildFromConstants();
        }

        if (!built) {
            return ConstantsConf.Shooter.RPM_NEAR; // fallback
        }

        // Garante que min/max estão válidos (evita 0 se build falhar parcialmente)
        double min = minDistance > 0 ? minDistance : ConstantsConf.Shooter.DIST_NEAR_POL;
        double max = maxDistance >= min ? maxDistance : min;

        // Clamp: distância 0 ou negativa (ex.: pose não inicializada) não pode ir para a LUT
        double clampedDistance = distanceInches;
        if (distanceInches < min || !Double.isFinite(distanceInches)) {
            clampedDistance = min;
        } else if (distanceInches > max) {
            clampedDistance = max;
        }

        return lut.get(clampedDistance);
    }

    public void invalidate() {
        built = false;
    }
}
