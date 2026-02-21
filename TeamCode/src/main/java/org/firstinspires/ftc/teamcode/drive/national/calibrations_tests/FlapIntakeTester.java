package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * TeleOp integrado para testar Flywheel, Intake e Flap juntos.
 *
 * Controles:
 * - Left Trigger: Toggle intake (liga/desliga)
 * - Right Trigger: Flap - vai para posição de alinhamento, fica 2s, depois volta (igual TeleOp)
 * - D-Pad Up/Down: Aumenta/diminui velocidade do shooter
 * - X: Alterna scale factor do D-Pad (1x, 10x, 100x)
 * - A: Ativa shooter com velocidade atual
 *
 * Configure os motores e servo no Robot Configuration.
 */

@TeleOp(name = "Flap Intake Flywheel Tester", group = "Tuning")
public class FlapIntakeTester extends OpMode {

    private Servo flapServo;
    private DcMotorEx intakeMotor;
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;

    // Intake toggle state
    private boolean intakeActive = false;
    private boolean leftTriggerPrev = false;
    private boolean rightTriggerPrev = false;

    // Flap state machine (igual ao IntakeSubsystem)
    private enum FlapState {
        NORMAL,      // Posição padrão (0.0)
        ALIGNING,    // Movendo para posição alinhada (1.0)
        HOLDING,     // Mantendo na posição alinhada por 2 segundos
        RETURNING    // Voltando para posição normal
    }
    private FlapState flapState = FlapState.NORMAL;
    private ElapsedTime flapTimer = new ElapsedTime();
    private static final double FLAP_ALIGNED_POSITION = 1.0;
    private static final double FLAP_NORMAL_POSITION = 0.0;
    private static final double FLAP_HOLD_TIME = 2.0;

    // Shooter control
    private double curTargetVelocity = 0;
    private boolean shooterActive = false;
    private boolean aPrev = false;

    // Scale factor para D-Pad
    private double scaleFactor = 50.0; // Padrão: 50 ticks/s por pressão
    private boolean xPrev = false;
    private int scaleMode = 0; // 0 = 50, 1 = 500, 2 = 5000

    @Override
    public void init() {
        // Initialize flap servo
        try {
            flapServo = hardwareMap.get(Servo.class, "flap");
            flapServo.setPosition(FLAP_NORMAL_POSITION);
            telemetry.addData("Flap", "Servo encontrado");
        } catch (Exception e) {
            flapServo = null;
            telemetry.addData("Flap", "Servo NAO encontrado");
        }

        // Initialize intake motor
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, ConstantsConf.Intake.INTAKE_MOTOR_NAME);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            telemetry.addData("Intake", "Motor encontrado: %s", ConstantsConf.Intake.INTAKE_MOTOR_NAME);
        } catch (Exception e) {
            intakeMotor = null;
            telemetry.addData("Intake", "Motor NAO encontrado");
        }

        // Initialize flywheel motors
        try {
            leftFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_LEFT_MOTOR_NAME);
            rightFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_RIGHT_MOTOR_NAME);

            leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

            PIDFCoefficients pidfCoefficients = new PIDFCoefficients(
                ConstantsConf.Shooter.KP,
                ConstantsConf.Shooter.KI,
                ConstantsConf.Shooter.KD,
                ConstantsConf.Shooter.KF
            );
            leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
            telemetry.addData("Shooter", "Motores encontrados");
        } catch (Exception e) {
            leftFlywheel = null;
            rightFlywheel = null;
            telemetry.addData("Shooter", "Motores NAO encontrados");
        }

        telemetry.addLine();
        telemetry.addData("--- Controles ---", "");
        telemetry.addData("Left Trigger", "Toggle intake");
        telemetry.addData("Right Trigger", "Flap: alinhar -> 2s -> voltar");
        telemetry.addData("D-Pad Up/Down", "Ajustar velocidade shooter");
        telemetry.addData("X", "Alternar scale factor (50/500/5000)");
        telemetry.addData("A", "Ativar shooter");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Intake toggle (left trigger) - igual ao TeleOp
        boolean leftTriggerNow = gamepad1.left_trigger > 0.1;
        if (leftTriggerNow && !leftTriggerPrev) {
            intakeActive = !intakeActive;
        }
        leftTriggerPrev = leftTriggerNow;

        if (intakeMotor != null) {
            if (intakeActive) {
                intakeMotor.setPower(ConstantsConf.Intake.INTAKE_POWER);
            } else {
                intakeMotor.setPower(0.0);
            }
        }

        // Flap control (right trigger) - igual ao TeleOp
        boolean rightTriggerNow = gamepad1.right_trigger > 0.1;
        if (rightTriggerNow && !rightTriggerPrev && flapState == FlapState.NORMAL) {
            flapState = FlapState.ALIGNING;
            flapTimer.reset();
            if (flapServo != null) {
                flapServo.setPosition(FLAP_ALIGNED_POSITION);
            }
        }
        rightTriggerPrev = rightTriggerNow;

        // Update flap state machine
        updateFlap();

        // Shooter velocity control (D-Pad Up/Down)
        if (leftFlywheel != null && rightFlywheel != null) {
            if (gamepad1.dpad_up) {
                curTargetVelocity += scaleFactor;
                curTargetVelocity = Math.min(6000, curTargetVelocity);
            }
            if (gamepad1.dpad_down) {
                curTargetVelocity -= scaleFactor;
                curTargetVelocity = Math.max(0, curTargetVelocity);
            }

            // Scale factor toggle (X button)
            boolean xNow = gamepad1.x;
            if (xNow && !xPrev) {
                scaleMode = (scaleMode + 1) % 3;
                switch (scaleMode) {
                    case 0:
                        scaleFactor = 50.0;
                        break;
                    case 1:
                        scaleFactor = 500.0;
                        break;
                    case 2:
                        scaleFactor = 5000.0;
                        break;
                }
            }
            xPrev = xNow;

            // Activate shooter (A button)
            boolean aNow = gamepad1.a;
            if (aNow && !aPrev) {
                shooterActive = !shooterActive;
            }
            aPrev = aNow;

            if (shooterActive) {
                leftFlywheel.setVelocity(curTargetVelocity);
                rightFlywheel.setVelocity(curTargetVelocity);
            } else {
                leftFlywheel.setVelocity(0);
                rightFlywheel.setVelocity(0);
            }
        }

        // Telemetry
        telemetry.clear();

        telemetry.addLine("=== FLYWHEEL SHOOTER ===");
        if (leftFlywheel != null && rightFlywheel != null) {
            double curVelocityLeft = leftFlywheel.getVelocity();
            double curVelocityRight = rightFlywheel.getVelocity();
            double curVelocityAvg = (curVelocityLeft + curVelocityRight) / 2.0;
            double error = curTargetVelocity - curVelocityAvg;

            telemetry.addData("Target Velocity", "%.0f ticks/s", curTargetVelocity);
            telemetry.addData("Current Left", "%.0f ticks/s", curVelocityLeft);
            telemetry.addData("Current Right", "%.0f ticks/s", curVelocityRight);
            telemetry.addData("Current Avg", "%.0f ticks/s", curVelocityAvg);
            telemetry.addData("Error", "%.0f ticks/s", error);
            telemetry.addData("Active (A)", shooterActive ? "YES" : "NO");
            telemetry.addData("Scale Factor (X)", "%.0f", scaleFactor);
        } else {
            telemetry.addData("Status", "Motores NAO disponiveis");
        }

        telemetry.addLine();
        telemetry.addLine("=== INTAKE ===");
        if (intakeMotor != null) {
            telemetry.addData("Active (LT)", intakeActive ? "ON" : "OFF");
            telemetry.addData("Power", "%.2f", intakeActive ? ConstantsConf.Intake.INTAKE_POWER : 0.0);
        } else {
            telemetry.addData("Status", "Motor NAO disponivel");
        }

        telemetry.addLine();
        telemetry.addLine("=== FLAP SERVO ===");
        if (flapServo != null) {
            String stateStr = "";
            switch (flapState) {
                case NORMAL:
                    stateStr = "NORMAL";
                    break;
                case ALIGNING:
                    stateStr = "ALINHANDO";
                    break;
                case HOLDING:
                    stateStr = String.format("SEGURANDO (%.1fs)", FLAP_HOLD_TIME - flapTimer.seconds());
                    break;
                case RETURNING:
                    stateStr = "VOLTANDO";
                    break;
            }
            telemetry.addData("State", stateStr);
            telemetry.addData("Position", "%.2f", flapServo.getPosition());
            telemetry.addData("Control (RT)", "Pressione para ciclo");
        } else {
            telemetry.addData("Status", "Servo NAO disponivel");
        }

        telemetry.addLine();
        telemetry.addLine("=== CONTROLES ===");
        telemetry.addData("LT", "Toggle intake");
        telemetry.addData("RT", "Flap cycle");
        telemetry.addData("D-Pad U/D", "Velocidade shooter");
        telemetry.addData("X", "Scale: " + scaleFactor);
        telemetry.addData("A", "Toggle shooter");

        telemetry.update();
    }

    /**
     * Update flap state machine (igual ao IntakeSubsystem).
     */
    private void updateFlap() {
        if (flapServo == null) return;

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
                    flapServo.setPosition(FLAP_NORMAL_POSITION);
                }
                break;

            case RETURNING:
                if (flapTimer.seconds() > 0.25) {
                    flapState = FlapState.NORMAL;
                }
                break;

            case NORMAL:
            default:
                flapServo.setPosition(FLAP_NORMAL_POSITION);
                break;
        }
    }

    @Override
    public void stop() {
        if (flapServo != null) {
            flapServo.setPosition(FLAP_NORMAL_POSITION);
        }
        if (intakeMotor != null) {
            intakeMotor.setPower(0.0);
        }
        if (leftFlywheel != null) {
            leftFlywheel.setVelocity(0);
        }
        if (rightFlywheel != null) {
            rightFlywheel.setVelocity(0);
        }
    }
}
