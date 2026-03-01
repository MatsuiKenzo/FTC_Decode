package org.firstinspires.ftc.teamcode.drive.national.objects;

/**
 * Zonas do campo para comportamento da turret: dentro da zona mira no goal, fora trava em 180°.
 * Polígonos definidos pelas coordenadas (X, Y) do esboço do campo.
 */
public class ShootingZones {

    /** Zona do Red Goal (4 pontos). Dentro desta ou da zona azul, TeleOp Red mira no Red Goal; TeleOp Blue no Blue Goal. */
    private static final double[][] RED_GOAL_ZONE = {
            { 71.7757009,  42.392523 },
            { 38.13084112,  0.89719626 },
            { 71.38317757,  0.7943925 },
            { 105.925233,   0.01869158 }
    };

    /** Zona do Blue Goal (7 pontos). Mesmo alvo por aliança: Red → Red Goal, Blue → Blue Goal. */
    private static final double[][] BLUE_GOAL_ZONE = {
            { 142.822429,  119.6728971 },
            { 143.205607,  143.102803 },
            { 0,           144 },
            { 0.22429901,  119.570093 },
            { 23.3738317,  96.242990 },
            { 72,          58.766355 },
            { 120.205607,  95.9813084 }
    };

    /** Centro do Blue Goal. */
    private static final double BLUE_GOAL_X = 12.167;
    private static final double BLUE_GOAL_Y = 136.204;

    /** Centro do Red Goal. */
    private static final double RED_GOAL_X = 131.270;
    private static final double RED_GOAL_Y = 135.531;

    /**
     * Verifica se o ponto (x, y) está dentro do polígono (ray casting).
     */
    public static boolean pointInPolygon(double x, double y, double[][] polygon) {
        int n = polygon.length;
        boolean inside = false;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            double xi = polygon[i][0], yi = polygon[i][1];
            double xj = polygon[j][0], yj = polygon[j][1];
            if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
                inside = !inside;
            }
        }
        return inside;
    }

    public static boolean isInRedGoalZone(double x, double y) {
        return pointInPolygon(x, y, RED_GOAL_ZONE);
    }

    public static boolean isInBlueGoalZone(double x, double y) {
        return pointInPolygon(x, y, BLUE_GOAL_ZONE);
    }

    /** Retorna true se o robô está em alguma zona de mira. */
    public static boolean isInAnyShootingZone(double x, double y) {
        return isInRedGoalZone(x, y) || isInBlueGoalZone(x, y);
    }

    /** X do Red Goal. */
    public static double getRedGoalX() { return RED_GOAL_X; }
    /** Y do Red Goal. */
    public static double getRedGoalY() { return RED_GOAL_Y; }

    /** X do Blue Goal. */
    public static double getBlueGoalX() { return BLUE_GOAL_X; }
    /** Y do Blue Goal. */
    public static double getBlueGoalY() { return BLUE_GOAL_Y; }
}
