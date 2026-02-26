package org.firstinspires.ftc.teamcode.drive.national.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Intake subsystem.
 *
 * Features:
 * - Controle de intake/outtake com toggle no left trigger
 * - Dois motores (intake + intake_2 opcional), mesma potência (igual Flap Intake Tester)
 * - Controle de servo da pá (flap) para alinhar bolas com shooter
 */
public class IntakeSubsystem {
    private DcMotorEx intakeMotor;
    private DcMotorEx intakeMotor2; // opcional, mesma potência quando ativo
    private Servo flapServo;
    private Servo flapServo2;

    // Toggle state para intake
    private boolean intakeActive = false;

    // Servo da pá (flap) - estados e controle
    private enum FlapState {
        NORMAL,      // Posição padrão (0.0)
        ALIGNING,    // Movendo para posição alinhada (1.0)
        HOLDING,     // Mantendo na posição alinhada por 2 segundos
        RETURNING    // Voltando para posição normal
    }
    private FlapState flapState = FlapState.NORMAL;
    private ElapsedTime flapTimer = new ElapsedTime();
    private static final double FLAP_ALIGNED_POSITION = 1.0;  // Posição alinhada com shooter
    private static final double FLAP_NORMAL_POSITION = 0.0;  // Posição padrão
    private static final double FLAP_HOLD_TIME = 2.0;  // Tempo em segundos para manter alinhado

    /**
     * Initialize the intake subsystem.
     *
     * @param hardwareMap HardwareMap from OpMode
     */
    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, ConstantsConf.Intake.INTAKE_MOTOR_NAME);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        try {
            intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake_2");
            intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD); // contrário ao intake_1 (REVERSE)
        } catch (Exception e) {
            intakeMotor2 = null;
        }

        // Servos da pá (flap) – os dois giram juntos
        try {
            flapServo = hardwareMap.get(Servo.class, ConstantsConf.Intake.FLAP_SERVO_NAME);
            flapServo.setPosition(FLAP_NORMAL_POSITION);
        } catch (Exception e) {
            flapServo = null;
        }
        try {
            flapServo2 = hardwareMap.get(Servo.class, ConstantsConf.Intake.FLAP2_SERVO_NAME);
            flapServo2.setPosition(FLAP_NORMAL_POSITION);
        } catch (Exception e) {
            flapServo2 = null;
        }
    }

    /** Aplica a mesma posição aos dois servos do flap (quando existirem). */
    private void setFlapPosition(double position) {
        if (flapServo != null) flapServo.setPosition(position);
        if (flapServo2 != null) flapServo2.setPosition(position);
    }

    /** Retorna true se pelo menos um servo do flap está disponível. */
    private boolean hasFlap() {
        return flapServo != null || flapServo2 != null;
    }


    /**
     * Set intake power.
     *
     * @param power Power [-1.0, 1.0]. Positive = intake, Negative = outtake
     */
    public void setPower(double power) {
        intakeMotor.setPower(power);
        if (intakeMotor2 != null) intakeMotor2.setPower(power);
    }

    /**
     * Legacy helper for older code that used a separate indexer motor.
     * Agora o indexer é o mesmo motor do intake.
     *
     * @param power Power [-1.0, 1.0]
     */
    public void setIndexerPower(double power) {
        setPower(power);
        intakeActive = Math.abs(power) > 1e-4;
    }

    /**
     * Toggle intake on/off usando left trigger como botão.
     * Clica uma vez ativa, clica de novo desativa.
     *
     * @param leftTriggerPressed true se o left trigger foi pressionado (detecta quando passa de não pressionado para pressionado)
     */
    public void toggleIntake(boolean leftTriggerPressed) {
        if (leftTriggerPressed) {
            intakeActive = !intakeActive;
        }
        
        // Aplica o estado do toggle ao motor
        if (intakeActive) {
            setPower(ConstantsConf.Intake.INTAKE_POWER);
        } else {
            setPower(0.0);
        }
    }

    /**
     * Get current intake toggle state.
     *
     * @return true se intake está ativo
     */
    public boolean isIntakeActive() {
        return intakeActive;
    }

    /**
     * Update servo da pá.
     * Chame este método no loop principal do OpMode.
     */
    public void update() {
        // Update servo da pá
        updateFlap();
    }

    /**
     * Controla o servo da pá (flap) para alinhar bolas com o shooter.
     * Ao pressionar o right trigger, executa ciclo: alinhar → 2s → voltar.
     *
     * @param triggerPressed true se o right trigger foi pressionado (detecta quando passa de não pressionado para pressionado)
     */
    public void shoot(boolean triggerPressed) {
        if (!hasFlap()) return;

        if (triggerPressed && flapState == FlapState.NORMAL) {
            flapState = FlapState.ALIGNING;
            flapTimer.reset();
            setFlapPosition(FLAP_ALIGNED_POSITION);
        }
    }

    /**
     * Update do servo da pá (chamar no loop principal).
     * Necessário para manter a máquina de estados funcionando.
     */
    public void updateFlap() {
        if (!hasFlap()) return;

        switch (flapState) {
            case ALIGNING:
                if (flapTimer.seconds() > 0.25) {
                    flapState = FlapState.HOLDING;
                    flapTimer.reset();
                }
                break;

            case HOLDING:
                if (flapTimer.seconds() >= FLAP_HOLD_TIME) {
                    flapState = FlapState.RETURNING;
                    flapTimer.reset();
                    setFlapPosition(FLAP_NORMAL_POSITION);
                }
                break;

            case RETURNING:
                if (flapTimer.seconds() > 0.25) {
                    flapState = FlapState.NORMAL;
                }
                break;

            case NORMAL:
            default:
                setFlapPosition(FLAP_NORMAL_POSITION);
                break;
        }
    }

    /**
     * Stop intake.
     */
    public void stop() {
        setPower(0.0);
        intakeActive = false;
        if (hasFlap()) {
            setFlapPosition(FLAP_NORMAL_POSITION);
            flapState = FlapState.NORMAL;
        }
    }
}
