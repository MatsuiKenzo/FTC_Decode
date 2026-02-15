package org.firstinspires.ftc.teamcode.drive.util;

/**
 * Constants class for robot configuration.
 *
 * This class contains all tunable constants for the robot subsystems.
 * Update these values based on your robot's hardware and tuning results.
 */
public class ConstantsConf {

    /**
     * Shooter subsystem constants.
     */
    public static class Shooter {
        // PID coefficients for velocity control
        // These values need to be tuned for your specific robot
        public static double KP = 0.01;   // Proportional gain
        public static double KI = 0.0001;  // Integral gain
        public static double KD = 0.0001;  // Derivative gain
        public static double KF = 0.0001;  // Feedforward gain

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

        // Motor configuration
        public static String FLYWHEEL_MOTOR_NAME = "flywheel";

        // Encoder configuration
        // If using REV encoder: 28 counts per revolution
        // If using goBILDA encoder: varies by model
        public static double TICKS_PER_REVOLUTION = 28.0;

        // Compensação de bateria: tensão de referência. Com bateria cheia o tiro fica mais forte;
        // abaixe este valor (ex: 11.5) para reduzir a potência quando a tensão estiver alta.
        public static double NOMINAL_VOLTAGE = 11;

        // Maximum RPM (for reference)
        public static double MAX_RPM = 6000.0;
    }

    /**
     * Intake subsystem constants.
     */
    public static class Intake {
        public static double INTAKE_POWER = 0.8;
        public static double OUTTAKE_POWER = -0.5;
        public static String INTAKE_MOTOR_NAME = "intake";
        public static String INDEXER_MOTOR_NAME = "index";
    }

    /**
     * Turret subsystem constants.
     */
    public static class Turret {
        public static double KP = 0.06;
        public static double KI = 0.0;
        public static double KD = 0.0005;
        public static double MIN_LIMIT = -60.0;
        public static double MAX_LIMIT = 260.0;
        public static String TURRET_MOTOR_NAME = "RMX";
    }

    /**
     * Drive subsystem constants.
     */
    public static class Drive {
        public static double MAX_SPEED = 1.0;
        public static double SLOW_SPEED = 0.5;
    }
}
