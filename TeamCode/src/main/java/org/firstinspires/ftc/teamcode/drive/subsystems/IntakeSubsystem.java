package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Intake subsystem com detecção de bola.
 *
 * Features:
 * - Controle de intake/outtake
 * - Detecção de bola usando sensor de distância
 * - Detecção de cor para identificar tipo de bola
 * - Controle de indexer integrado
 */
public class IntakeSubsystem {
    private DcMotorEx intakeMotor;
    private DcMotorEx indexerMotor;
    private DistanceSensor distanceSensor;
    private NormalizedColorSensor colorSensor;

    // Ball detection (distance sensor)
    private static final double BALL_DETECTION_DISTANCE_MM = 138.0; // mm
    // Color sensor: ball in position (trava a bola) - verde ou roxa
    private static final double COLOR_BALL_GREEN_THRESHOLD = 0.010;  // bola verde
    private static final double COLOR_BALL_PURPLE_RED_MIN  = 0.010;  // bola roxa: vermelho
    private static final double COLOR_BALL_PURPLE_BLUE_MIN = 0.010;  // bola roxa: azul
    private boolean hasBall = false;
    private String ballColorName = "NONE";

    /**
     * Initialize the intake subsystem.
     *
     * @param hardwareMap HardwareMap from OpMode
     */
    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, ConstantsConf.Intake.INTAKE_MOTOR_NAME);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Indexer (if exists)
        try {
            indexerMotor = hardwareMap.get(DcMotorEx.class, "index");
            indexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            indexerMotor = null;
        }

        // Distance sensor (if exists)
        try {
            distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        } catch (Exception e) {
            distanceSensor = null;
        }

        // Color sensor - mesmo nome do código antigo (ShooterObjBlue / ColorSensor)
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
    }

    /**
     * Set intake power.
     *
     * @param power Power [-1.0, 1.0]. Positive = intake, Negative = outtake
     */
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * Collect balls (intake).
     *
     * @param leftTrigger Left trigger value [0.0, 1.0]
     * @param rightTrigger Right trigger value [0.0, 1.0]
     */
    public void collect(double leftTrigger, double rightTrigger) {
        double power = (leftTrigger + rightTrigger) * 2.0;
        setPower(power);
    }

    /**
     * Set indexer power.
     *
     * @param power Power [-1.0, 1.0]
     */
    public void setIndexerPower(double power) {
        if (indexerMotor != null) {
            indexerMotor.setPower(power);
        }
    }

    /**
     * Update ball detection.
     * Uses color sensor: ball in position (trava a bola) when green ball OR purple ball detected.
     */
    public void update() {
        if (colorSensor != null) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            boolean isGreen = colors.green > COLOR_BALL_GREEN_THRESHOLD;
            boolean isPurple = colors.red >= COLOR_BALL_PURPLE_RED_MIN && colors.blue >= COLOR_BALL_PURPLE_BLUE_MIN;
            hasBall = isGreen || isPurple;
            ballColorName = isGreen ? "GREEN" : (isPurple ? "PURPLE" : "NONE");
        } else if (distanceSensor != null) {
            double distance = distanceSensor.getDistance(DistanceUnit.MM);
            hasBall = distance < BALL_DETECTION_DISTANCE_MM;
            ballColorName = hasBall ? "DETECTED" : "NONE";
        }
    }

    /**
     * Check if ball is detected.
     *
     * @return true if ball is detected
     */
    public boolean hasBall() {
        return hasBall;
    }

    /**
     * Get ball color / detection source for telemetry.
     *
     * @return "GREEN" when color sensor sees ball, "DETECTED" when distance only, "NONE" otherwise
     */
    public String getBallColor() {
        return ballColorName;
    }

    /**
     * Shoot ball (activate indexer).
     *
     * @param trigger Trigger value [0.0, 1.0]
     */
    public void shoot(double trigger) {
        if (trigger > 0.05) {
            // Shoot: push ball to shooter
            setIndexerPower(1.0);
        } else if (!hasBall) {
            // No ball: advance indexer
            setIndexerPower(0.7);
        } else {
            // Ball detected: stop indexer
            setIndexerPower(0.0);
        }
    }

    /**
     * Stop intake.
     */
    public void stop() {
        setPower(0.0);
        setIndexerPower(0.0);
    }

    /**
     * Get distance sensor reading.
     *
     * @return Distance in mm, or -1 if sensor not available
     */
    public double getDistance() {
        if (distanceSensor == null) return -1.0;
        return distanceSensor.getDistance(DistanceUnit.MM);
    }
}
