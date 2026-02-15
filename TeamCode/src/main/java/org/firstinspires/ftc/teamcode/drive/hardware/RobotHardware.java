package org.firstinspires.ftc.teamcode.drive.hardware;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.PedroPathingShooter;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

/**
 * Robot hardware configuration class.
 *
 * This class initializes and manages all hardware components of the robot.
 *
 * Integrates PedroPathing for localization and target tracking.
 */
public class RobotHardware {
    // Subsystems
    public PedroPathingShooter shooter;
    public TurretSubsystem turret;
    public IntakeSubsystem intake;

    // PedroPathing Follower (must be initialized externally)
    private Follower follower;

    // Hardware map
    private HardwareMap hardwareMap;

    // Motor names
    private static final String TURRET_MOTOR_NAME = "RMX";
    private static final String SHOOTER_MOTOR_NAME = "RMTa";

    /**
     * Initialize all robot hardware without PedroPathing.
     *
     * @param hardwareMap HardwareMap from OpMode
     */
    public RobotHardware(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    /**
     * Initialize all robot hardware.
     *
     * @param hardwareMap HardwareMap from OpMode
     * @param follower PedroPathing Follower instance (can be null if not using PedroPathing)
     */
    public RobotHardware(HardwareMap hardwareMap, Follower follower) {
        this.hardwareMap = hardwareMap;
        this.follower = follower;

        // Initialize subsystems
        initializeSubsystems();
    }

    /**
     * Initialize all subsystems.
     */
    private void initializeSubsystems() {
        // Initialize shooter - PedroPathingShooter works with or without follower
        shooter = new PedroPathingShooter(hardwareMap, follower, SHOOTER_MOTOR_NAME);
        shooter.setPowerConfig(0.35, 0.9, 20.0, 115.0); // Default config

        // Initialize turret only if follower is available
        if (follower != null) {
            turret = new TurretSubsystem();
            turret.init(hardwareMap, follower, TURRET_MOTOR_NAME);
            turret.setLimits(-60.0, 260.0);
        }

        // Initialize intake
        intake = new IntakeSubsystem(hardwareMap);
    }

    /**
     * Set target position for shooter and turret.
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
            // Turret doesn't have a stop method, but we can set power to 0
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
