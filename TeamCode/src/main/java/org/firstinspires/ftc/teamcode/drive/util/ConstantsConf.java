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

        // Dois servos contínuos da turret — giram juntos (mesma potência nos dois)
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
        public static double TURRET_MIN_LIMIT = -180.0;
        public static double TURRET_MAX_LIMIT = 180.0;

        /** Graus por segundo por unidade de potência (relação engrenagem). Use TurretCalibrator para calibrar. */
        public static double TURRET_DEGREES_PER_SECOND_PER_POWER = 193.3;

        /** Offset do heading do robô em graus (ex: 180 = frente ao contrário no Field Oriented Drive). Aplicado na IMU. */
        public static double DRIVE_HEADING_OFFSET_DEG = 180.0;

        // ---------- Turret com encoder (REV Through Bore Encoder V1) ----------
        /** Se true, usa encoder na turret para ângulo real; senão usa estimativa tempo×potência. */
        public static boolean TURRET_ENCODER_ENABLED = true;
        /**
         * Nome do DcMotorEx no config de onde LER a posição do encoder da torreta.
         * - Canal livre: use "turret_encoder" (encoder na porta de encoder; motor desse canal desconectado).
         * - Todos os canais em uso: use "intake" (ou "intake_2"); conecte o encoder da torreta na
         *   porta de encoder desse canal; a saída do motor continua no motor do intake (intake não usa encoder).
         */
        public static String TURRET_ENCODER_MOTOR_NAME = "intake";
        /** REV Through Bore Encoder V1 (REV-11-1271): 8192 counts/rev do eixo do encoder (quadrature). */
        public static int TURRET_ENCODER_TICKS_PER_REV = 8192;
        /** Engrenagem da base giratória (turret): número de dentes. */
        public static int TURRET_ENCODER_GEAR_TURRET_TEETH = 180;
        /** Engrenagem no eixo do encoder: número de dentes. */
        public static int TURRET_ENCODER_GEAR_ENCODER_TEETH = 87;
        /** Sentido: 1.0 ou -1.0. Se com potência negativa o ângulo sobe na telemetria, use -1. */
        public static double TURRET_ENCODER_DIRECTION = -1.0;
        /** Deadband da turret (graus): abaixo deste erro o PID retorna 0 para evitar oscilação. Ajuste (ex: 1.0–2.5) se tremer. */
        public static double TURRET_DEADBAND_DEG = 1.5;
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
        public static double KP = 20;   // Proportional gain
        public static double KI = 0.000001;  // Integral gain
        public static double KD = 0.0001;  // Derivative gain
        public static double KF = 16.9;  // Feedforward gain

        // Calibração distância → RPM (Shooter Tuner)
        public static double DIST_NEAR_POL = 63.35;
        public static double RPM_NEAR = 1600;
        public static double DIST_MID_POL = 87;
        public static double RPM_MID = 1721;
        public static double DIST_116_POL = 116.4;
        public static double RPM_116 = 2786;
        public static double DIST_FAR_POL = 134.53;
        public static double RPM_FAR = 4437;

        /**
         * Pontos para a LUT distância → RPM (interpolação linear em ShooterDistanceToRPM).
         * Ordene por distância crescente. Quanto mais pontos, mais precisa a curva.
         * Ex.: 3 pontos = perto/meio/longe; 5–7 pontos = calibração mais fina.
         *
         * Dicas de calibração:
         * - Use o Linear Interpolation Tuner: posicione o robô em distâncias conhecidas (ex.: 63, 99, 145 pol),
         *   ajuste RPM até o tiro ficar bom e anote (distância, RPM).
         * - Preencha DISTANCE_LUT_POL e RPM_LUT com os mesmos índices (distância[i] ↔ RPM[i]).
         * - Mais pontos no meio da faixa de tiro melhoram a precisão; evite só 2 pontos.
         */
        public static double[] DISTANCE_LUT_POL = { DIST_NEAR_POL, DIST_MID_POL, DIST_116_POL, DIST_FAR_POL };
        public static double[] RPM_LUT = { RPM_NEAR, RPM_MID, RPM_116, RPM_FAR };

        // Fallback para Shooter Test / quando não usa distância (ticks/s = RPM * 28 / 60)
        public static double LOW_VELOCITY = 1000;   // 3062 RPM
        public static double MEDIUM_VELOCITY = 1400;  // 3686 RPM
        public static double HIGH_VELOCITY = 1983.33;   // 4250 RPM

        // Motor configuration (usado apenas pelo sistema Regional)
        public static String FLYWHEEL_MOTOR_NAME = "flywheel";

        // Encoder configuration
        // Motor Yellow Jacket Série 5203 = 28 ticks_per_revolution
        public static double TICKS_PER_REVOLUTION = 28.0;

        // Compensação de bateria: tensão de referência. Com bateria cheia o tiro fica mais forte;
        // abaixar este valor (ex: 11.0) para reduzir a potência quando a tensão estiver alta.
        public static double NOMINAL_VOLTAGE = 12.2;

        // Maximum RPM (for reference)
        public static double MAX_RPM = 6000.0;
    }

    /**
     * Intake subsystem constants.
     * Compartilhado entre Nacional e Regional.
     */
    public static class Intake {
        public static double INTAKE_POWER = 1.0;
        public static double OUTTAKE_POWER = -0.5;
        public static String INTAKE_MOTOR_NAME = "intake";
        public static String INDEXER_MOTOR_NAME = "index";
        /** Servo da pá (flap) – primeiro. */
        public static String FLAP_SERVO_NAME = "flap_1";
        /** Servo da pá (flap) – segundo (gira junto com o primeiro). */
        public static String FLAP2_SERVO_NAME = "flap_2";
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
