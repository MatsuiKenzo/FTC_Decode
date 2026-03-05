package org.firstinspires.ftc.teamcode.drive.util;

import com.seattlesolvers.solverslib.util.LUT;

/**
 * Curva distância (pol) → RPM do shooter via LUT da SolversLib.
 *
 * Usa {@link LUT}{@code <Double, Double>}; os pontos vêm de
 * ConstantsConf.Shooter (DISTANCE_LUT_POL e RPM_LUT).
 * getRPM retorna o RPM do ponto mais próximo (getClosest).
 */
public class ShooterDistanceToRPM {

    private final LUT<Double, Double> rpmLut = new LUT<Double, Double>() {};
    private boolean built = false;

    /**
     * Constrói a LUT a partir dos pontos em ConstantsConf.Shooter.
     */
    public void buildFromConstants() {
        double[] dist = ConstantsConf.Shooter.DISTANCE_LUT_POL;
        double[] rpm = ConstantsConf.Shooter.RPM_LUT;

        if (dist == null || rpm == null || dist.length != rpm.length || dist.length == 0) {
            built = false;
            return;
        }

        rpmLut.clear();
        for (int i = 0; i < dist.length; i++) {
            rpmLut.add(dist[i], rpm[i]);
        }
        built = true;
    }

    /**
     * Retorna o RPM do ponto mais próximo da distância (polegadas).
     */
    public double getRPM(double distanceInches) {
        if (!built) {
            buildFromConstants();
        }
        if (!built || rpmLut.size() == 0) {
            return ConstantsConf.Shooter.RPM_NEAR;
        }
        if (!Double.isFinite(distanceInches)) {
            return ConstantsConf.Shooter.RPM_NEAR;
        }
        return rpmLut.getClosest(distanceInches);
    }

    public void invalidate() {
        built = false;
        rpmLut.clear();
    }
}
