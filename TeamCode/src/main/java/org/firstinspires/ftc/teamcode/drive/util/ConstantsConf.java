package org.firstinspires.ftc.teamcode.drive.util;

/**
 * Constants class for robot configuration.
 *
 * Esta classe contém todas as constantes configuráveis do robô.
 * Organizada por sistema: Nacional (principal) no topo, Regional/Qualifiers no final.
 *
 * Update these values based on your robot's hardware and tuning results.
 */
public class ConstantsConf {

    // ============================================================================
    // SISTEMA NACIONAL (Robô atual - 2 flywheels, 2 servos turret)
    // ============================================================================

    /**
     * Sistema Nacional - Shooter e Turret com dois motores/servos.
     * Este é o sistema principal usado no robô atual.
     */
    public static class Nacional {
        // Nomes dos motores do shooter nacional (dois flywheels)
        public static String SHOOTER_LEFT_MOTOR_NAME = "shooter_left";
        public static String SHOOTER_RIGHT_MOTOR_NAME = "shooter_right";

        // Nomes dos servos da turret nacional (dois servos contínuos)
        public static String TURRET_LEFT_SERVO_NAME = "turret_left";
        public static String TURRET_RIGHT_SERVO_NAME = "turret_right";

        // Nome do servo de tilt/hood (opcional - pode não estar conectado)
        public static String HOOD_SERVO_NAME = "hood";

        /**
         * ATIVAR/DESATIVAR TILT/HOOD
         * 
         * Se você NÃO tem o servo Taura conectado no Control Hub:
         * - Defina TILT_ENABLED = false
         * - OU não configure o servo "hood" no Robot Configuration
         * 
         * O código vai detectar automaticamente e desativar o tilt se o servo não for encontrado.
         */
        public static boolean TILT_ENABLED = false; // Mude para true quando conectar o servo

        // Limites da turret nacional (em graus relativos ao robô)
        public static double TURRET_MIN_LIMIT = -60.0;
        public static double TURRET_MAX_LIMIT = 260.0;
    }

    // ============================================================================
    // CONSTANTES COMPARTILHADAS (usadas por Nacional e Regional)
    // ============================================================================

    /**
     * Shooter subsystem constants.
     * Usado por ambos os sistemas (Nacional e Regional).
     * O sistema Nacional usa estes valores para ambos os motores de flywheel.
     */
    public static class Shooter {
        // PID coefficients for velocity control
        // These values need to be tuned for your specific robot
        public static double KP = 40.0;   // Proportional gain
        public static double KI = 0.000001;  // Integral gain
        public static double KD = 0.0001;  // Derivative gain
        public static double KF = 15.0;  // Feedforward gain

        // Calibração distância → RPM (Shooter Tuner): perto, meio, longe
        public static double DIST_NEAR_POL = 63.1;
        public static double RPM_NEAR = 3062;
        public static double DIST_MID_POL = 98.7;
        public static double RPM_MID = 3686;
        public static double DIST_FAR_POL = 145.5;
        public static double RPM_FAR = 4250;

        // Fallback para Shooter Test / quando não usa distância (ticks/s = RPM * 28 / 60)
        public static double LOW_VELOCITY = 1428.93;   // 3062 RPM
        public static double MEDIUM_VELOCITY = 1718.27;  // 3686 RPM
        public static double HIGH_VELOCITY = 1983.33;   // 4250 RPM

        // Motor configuration (usado apenas pelo sistema Regional)
        public static String FLYWHEEL_MOTOR_NAME = "flywheel";

        // Encoder configuration
        // Motor Yellow Jacket Série 5203 = 28 ticks_per_revolution
        public static double TICKS_PER_REVOLUTION = 28.0;

        // Compensação de bateria: tensão de referência. Com bateria cheia o tiro fica mais forte;
        // abaixar este valor (ex: 11.0) para reduzir a potência quando a tensão estiver alta.
        public static double NOMINAL_VOLTAGE = 11;

        // Maximum RPM (for reference)
        public static double MAX_RPM = 6000.0;
    }

    /**
     * Intake subsystem constants.
     * Compartilhado entre Nacional e Regional.
     */
    public static class Intake {
        public static double INTAKE_POWER = 0.8;
        public static double OUTTAKE_POWER = -0.5;
        public static String INTAKE_MOTOR_NAME = "intake";
        public static String INDEXER_MOTOR_NAME = "index";
    }

    /**
     * Drive subsystem constants.
     * Compartilhado entre Nacional e Regional.
     */
    public static class Drive {
        public static double MAX_SPEED = 1.0;
        public static double SLOW_SPEED = 0.5;
    }

    /**
     * Localização com Limelight + Pinpoint (filtro de Kalman simplificado).
     * Requer Limelight3A configurado como "limelight" no Robot Configuration.
     * Compartilhado entre Nacional e Regional.
     */
    public static class KalmanLocalizer {
        /** Ativar fusão Pinpoint + Limelight. Desativar se não tiver Limelight. */
        public static boolean ENABLED = true;
    }

    // ============================================================================
    // SISTEMA REGIONAL / QUALIFIERS (Robô antigo - 1 flywheel, 1 motor turret)
    // ============================================================================
    // NOTA: Mantido apenas para compatibilidade.

    /**
     * Turret subsystem constants.
     * Usado APENAS pelo sistema Regional/Qualifiers (robô antigo).
     * O sistema Nacional usa NacionalTurret com dois servos contínuos.
     */
    public static class Turret {
        public static double KP = 0.06;
        public static double KI = 0.0;
        public static double KD = 0.0005;
        public static double MIN_LIMIT = -60.0;
        public static double MAX_LIMIT = 260.0;
        public static String TURRET_MOTOR_NAME = "RMX";
    }
}
