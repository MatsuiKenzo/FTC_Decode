package org.firstinspires.ftc.teamcode.drive.util;

/**
 * Curva distância → RPM do shooter por interpolação linear entre pontos.
 *
 * Usa interpolação linear própria (sem biblioteca externa): limites inclusivos,
 * sem exceção quando a distância é exatamente igual ao mínimo ou máximo da LUT.
 * Os pontos vêm de ConstantsConf.Shooter (DISTANCE_LUT_POL e RPM_LUT).
 */
public class ShooterDistanceToRPM {

    private double[] distances;
    private double[] rpms;
    private int n;
    private boolean built = false;

    /**
     * Constrói a LUT a partir dos pontos em ConstantsConf.Shooter.
     * Ordena por distância crescente internamente.
     */
    public void buildFromConstants() {
        double[] dist = ConstantsConf.Shooter.DISTANCE_LUT_POL;
        double[] rpm = ConstantsConf.Shooter.RPM_LUT;

        if (dist == null || rpm == null || dist.length != rpm.length || dist.length == 0) {
            built = false;
            return;
        }

        n = dist.length;
        distances = new double[n];
        rpms = new double[n];
        for (int i = 0; i < n; i++) {
            distances[i] = dist[i];
            rpms[i] = rpm[i];
        }
        sortByDistance();
        built = true;
    }

    private void sortByDistance() {
        for (int i = 0; i < n - 1; i++) {
            for (int j = i + 1; j < n; j++) {
                if (distances[j] < distances[i]) {
                    double td = distances[i]; distances[i] = distances[j]; distances[j] = td;
                    double tr = rpms[i]; rpms[i] = rpms[j]; rpms[j] = tr;
                }
            }
        }
    }

    /**
     * Retorna o RPM interpolado para a distância (polegadas).
     * Limites inclusivos: distância exatamente no mínimo ou máximo não gera exceção.
     */
    public double getRPM(double distanceInches) {
        if (!built) {
            buildFromConstants();
        }
        if (!built || n == 0) {
            return ConstantsConf.Shooter.RPM_NEAR;
        }

        double d = distanceInches;
        if (!Double.isFinite(d)) {
            d = distances[0];
        }
        if (d <= distances[0]) {
            return rpms[0];
        }
        if (d >= distances[n - 1]) {
            return rpms[n - 1];
        }

        for (int i = 0; i < n - 1; i++) {
            if (d >= distances[i] && d <= distances[i + 1]) {
                double t = (distances[i + 1] - distances[i]) == 0 ? 0
                        : (d - distances[i]) / (distances[i + 1] - distances[i]);
                return rpms[i] + t * (rpms[i + 1] - rpms[i]);
            }
        }
        return rpms[n - 1];
    }

    public void invalidate() {
        built = false;
    }
}
