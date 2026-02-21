package org.firstinspires.ftc.teamcode.drive.national.hardware;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.national.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.national.subsystems.NacionalHoodController;
import org.firstinspires.ftc.teamcode.drive.national.subsystems.NacionalShooter;
import org.firstinspires.ftc.teamcode.drive.national.subsystems.NacionalTurret;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Robot hardware configuration class para o sistema NACIONAL.
 *
 * Esta classe inicializa e gerencia todos os componentes de hardware do robô nacional.
 *
 * Integra PedroPathing para localização e rastreamento de alvos.
 *
 * Sistema Nacional:
 * - Shooter com DOIS motores de flywheel
 * - Turret com DOIS servos contínuos
 * - Hood/Tilt opcional (pode ser desativado se não tiver servo conectado)
 */
public class RobotHardwareNacional {
    // Subsystems
    public NacionalShooter shooter;
    public NacionalTurret turret;
    public NacionalHoodController hood; // Opcional - pode ser null se não conectado
    public IntakeSubsystem intake;

    // PedroPathing Follower (must be initialized externally)
    private Follower follower;

    // Hardware map
    private HardwareMap hardwareMap;

    /** Se true, inicializa a turret. Use false para teste sem servos da turret. */
    private boolean initTurret = true;

    /**
     * Initialize all robot hardware (inclui turret).
     *
     * @param hardwareMap HardwareMap from OpMode
     * @param follower PedroPathing Follower instance (can be null if not using PedroPathing)
     */
    public RobotHardwareNacional(HardwareMap hardwareMap, Follower follower) {
        this.hardwareMap = hardwareMap;
        this.follower = follower;
        this.initTurret = true;
        initializeSubsystems();
    }

    /**
     * Initialize all robot hardware (turret opcional).
     *
     * @param hardwareMap HardwareMap from OpMode
     * @param follower PedroPathing Follower instance (can be null if not using PedroPathing)
     * @param initTurret false para não inicializar turret (ex.: teste sem servos conectados)
     */
    public RobotHardwareNacional(HardwareMap hardwareMap, Follower follower, boolean initTurret) {
        this.hardwareMap = hardwareMap;
        this.follower = follower;
        this.initTurret = initTurret;
        initializeSubsystems();
    }

    /**
     * Initialize all subsystems.
     */
    private void initializeSubsystems() {
        // Initialize shooter nacional - DOIS motores
        shooter = new NacionalShooter(
                hardwareMap,
                follower,
                ConstantsConf.Nacional.SHOOTER_LEFT_MOTOR_NAME,
                ConstantsConf.Nacional.SHOOTER_RIGHT_MOTOR_NAME
        );
        shooter.setPowerConfig(0.35, 0.9, 20.0, 115.0); // Default config

        // Initialize turret nacional - DOIS servos contínuos (opcional)
        if (initTurret && follower != null) {
            turret = new NacionalTurret();
            turret.init(
                    hardwareMap,
                    follower,
                    ConstantsConf.Nacional.TURRET_LEFT_SERVO_NAME,
                    ConstantsConf.Nacional.TURRET_RIGHT_SERVO_NAME
            );
            turret.setLimits(
                    ConstantsConf.Nacional.TURRET_MIN_LIMIT,
                    ConstantsConf.Nacional.TURRET_MAX_LIMIT
            );
        } else {
            turret = null;
        }

        // Initialize hood/tilt (OPCIONAL - pode não estar conectado)
        if (ConstantsConf.Nacional.TILT_ENABLED && follower != null) {
            hood = new NacionalHoodController();
            boolean hoodInitialized = hood.init(
                    hardwareMap,
                    follower,
                    ConstantsConf.Nacional.HOOD_SERVO_NAME
            );
            if (hoodInitialized) {
                hood.setDistanceBounds(20.0, 120.0);
                hood.setAngleBounds(0.40, 0.90); // Ajuste conforme necessário
            } else {
                // Servo não encontrado - desativa silenciosamente
                hood = null;
            }
        } else {
            hood = null;
        }

        // Initialize intake
        intake = new IntakeSubsystem(hardwareMap);
    }

    /**
     * Set target position for shooter, turret, and hood.
     *
     * @param x Target X coordinate (inches)
     * @param y Target Y coordinate (inches)
     */
    public void setTargetPosition(double x, double y) {
        if (shooter != null) {
            shooter.setTargetPosition(x, y);
        }
        if (turret != null) {
            turret.setTargetPosition(x, y);
        }
        if (hood != null && hood.isEnabled()) {
            hood.setTargetPosition(x, y);
        }
    }

    /**
     * Update all subsystems.
     * Call at OpMode's loop() method.
     */
    public void update() {
        if (shooter != null) {
            shooter.update();
        }
        if (turret != null) {
            turret.update();
        }
        if (hood != null && hood.isEnabled()) {
            hood.update();
        }
        if (intake != null) {
            intake.update();
        }
    }

    /**
     * Stop all subsystems.
     */
    public void stop() {
        if (shooter != null) {
            shooter.stop();
        }
        if (turret != null) {
            turret.stopRotation();
        }
        if (hood != null && hood.isEnabled()) {
            hood.safePosition();
        }
        if (intake != null) {
            intake.stop();
        }
    }

    /**
     * Get PedroPathing Follower instance.
     *
     * @return Follower instance
     */
    public Follower getFollower() {
        return follower;
    }
}
