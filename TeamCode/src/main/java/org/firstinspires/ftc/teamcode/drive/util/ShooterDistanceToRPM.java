package org.firstinspires.ftc.teamcode.drive.util;

import com.seattlesolvers.solverslib.util.InterpLUT;

/**
 * Encapsula a curva distância → RPM do shooter usando InterpLUT (SolversLib/FTCLib).
 * Permite calibração com vários pontos; a interpolação entre eles é feita pela própria LUT.
 *
 * Pontos de calibração vêm de ConstantsConf.Shooter (arrays DISTANCE_LUT_POL e RPM_LUT).
 * Para calibrar com mais precisão: use o Shooter Tuner, anote (distância, RPM) em vários
 * pontos e preencha os arrays em ConstantsConf. Quanto mais pontos, mais suave e preciso.
 */
public class ShooterDistanceToRPM {

    private InterpLUT lut;
    private boolean built = false;

    /**
     * Constrói a LUT a partir dos pontos definidos em ConstantsConf.Shooter.
     * Deve ser chamado após alterar os arrays de calibração (ou uma vez no init).
     */
    public void buildFromConstants() {
        double[] distances = ConstantsConf.Shooter.DISTANCE_LUT_POL;
        double[] rpms = ConstantsConf.Shooter.RPM_LUT;
        if (distances == null || rpms == null || distances.length != rpms.length || distances.length == 0) {
            built = false;
            return;
        }
        lut = new InterpLUT();
        for (int i = 0; i < distances.length; i++) {
            lut.add(distances[i], rpms[i]);
        }
        lut.createLUT();
        built = true;
    }

    /**
     * Retorna o RPM interpolado para a distância dada (em polegadas).
     * Valores fora do intervalo da LUT são interpolados/extrapolados pela InterpLUT.
     *
     * @param distanceInches distância ao alvo em polegadas
     * @return RPM alvo (interpolado)
     */
    public double getRPM(double distanceInches) {
        if (!built) {
            buildFromConstants();
        }
        if (!built) {
            return ConstantsConf.Shooter.RPM_NEAR; // fallback
        }
        return lut.get(distanceInches);
    }

    /** Invalida a LUT (próxima getRPM vai reconstruir a partir de ConstantsConf). */
    public void invalidate() {
        built = false;
    }
}
