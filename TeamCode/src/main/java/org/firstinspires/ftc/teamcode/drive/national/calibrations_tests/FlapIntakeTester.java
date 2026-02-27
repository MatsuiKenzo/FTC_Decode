package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.national.actuators.KalmanFilterLocalizer;
import org.firstinspires.ftc.teamcode.drive.national.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;
import org.firstinspires.ftc.teamcode.drive.util.ShooterDistanceToRPM;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * TeleOp integrado para testar Flywheel, Intake e Flap juntos.
 *
 * Intake: usa os dois motores (intake + intake_2) com a mesma potência no toggle.
 * Shooter: pode usar velocidade MANUAL (D-Pad) ou por DISTÂNCIA (pose do Kalman ao goal).
 *
 * Controles:
 * - Left Trigger: Toggle intake — intake e intake_2 juntos
 * - Right Trigger: Flap (alinhar → 2s → voltar)
 * - D-Pad Up/Down: Velocidade shooter (só em modo Manual)
 * - X: Alterna scale factor do D-Pad (10/100/1000)
 * - A: Ativar shooter
 * - B: Alternar modo shooter — Manual (D-Pad) ou Kalman (distância ao goal)
 *
 * Starting pose: igual TeleOp Blue Nacional (39, 80, 180°). Goal: (6, 140).
 * Robot Configuration: intake, intake_2, flap_1, flap_2, shooter, drive, imu, pinpoint, limelight (opcional).
 */

@TeleOp(name = "Flap Intake Flywheel Tester", group = "Tuning")
public class FlapIntakeTester extends OpMode {

    private Servo flapServo;
    private Servo flapServo2;
    private DcMotorEx intakeMotor;
    private DcMotorEx intakeMotor2;
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;

    // Pose / Kalman (starting pose igual TeleOp Blue Nacional)
    private Follower follower;
    private KalmanFilterLocalizer kalmanFilter;
    private FieldOrientedDrive fod;
    private static final Pose START_POSE = new Pose(39, 80, Math.toRadians(180));
    private static final double GOAL_X = 6.0;
    private static final double GOAL_Y = 140.0;
    private final ShooterDistanceToRPM distanceToRPM = new ShooterDistanceToRPM();

    // Modo shooter: false = Manual (D-Pad), true = Kalman (distância ao goal)
    private boolean useDistanceBasedVelocity = false;
    private boolean bPrev = false;
    private boolean yPrev = false;

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
        // Starting pose igual TeleOp Blue Nacional; drive + Kalman para distância ao goal
        fod = new FieldOrientedDrive(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_POSE);

        if (ConstantsConf.KalmanLocalizer.ENABLED) {
            kalmanFilter = new KalmanFilterLocalizer();
            if (kalmanFilter.init(hardwareMap, follower)) {
                telemetry.addData("Kalman", "Limelight + Pinpoint ativo (tags 20, 24)");
            } else {
                kalmanFilter = null;
                telemetry.addData("Kalman", "Limelight nao encontrada - usando so Pinpoint");
            }
        } else {
            kalmanFilter = null;
        }

        distanceToRPM.buildFromConstants();

        // Initialize flap servos (os dois giram juntos)
        try {
            flapServo = hardwareMap.get(Servo.class, ConstantsConf.Intake.FLAP_SERVO_NAME);
            flapServo.setPosition(FLAP_NORMAL_POSITION);
            telemetry.addData("Flap", "Servo '%s' encontrado", ConstantsConf.Intake.FLAP_SERVO_NAME);
        } catch (Exception e) {
            flapServo = null;
            telemetry.addData("Flap", "Servo NAO encontrado");
        }
        try {
            flapServo2 = hardwareMap.get(Servo.class, ConstantsConf.Intake.FLAP2_SERVO_NAME);
            flapServo2.setPosition(FLAP_NORMAL_POSITION);
            telemetry.addData("Flap2", "Servo '%s' encontrado", ConstantsConf.Intake.FLAP2_SERVO_NAME);
        } catch (Exception e) {
            flapServo2 = null;
        }

        // Initialize intake motors (intake + intake_2)
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, ConstantsConf.Intake.INTAKE_MOTOR_NAME);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            telemetry.addData("Intake", "Motor encontrado: %s", ConstantsConf.Intake.INTAKE_MOTOR_NAME);
        } catch (Exception e) {
            intakeMotor = null;
            telemetry.addData("Intake", "Motor NAO encontrado");
        }
        try {
            intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake_2");
            intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
            telemetry.addData("Intake2", "Motor 'intake_2' encontrado");
        } catch (Exception e) {
            intakeMotor2 = null;
            telemetry.addData("Intake2", "Motor 'intake_2' NAO encontrado");
        }

        // Initialize flywheel motors
        try {
            leftFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_LEFT_MOTOR_NAME);
            rightFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_RIGHT_MOTOR_NAME);

            leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);

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
        telemetry.addData("D-Pad Up/Down", "Velocidade shooter (modo Manual)");
        telemetry.addData("X", "Scale factor (10/100/1000)");
        telemetry.addData("A", "Ativar shooter");
        telemetry.addData("B", "Modo shooter: Manual <-> Kalman (distancia)");
        telemetry.addData("Y", "Reset pose -> (39, 80, 180°)");
        telemetry.addData("Sticks", "Dirigir (pose atualiza distancia)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Atualizar pose (Pinpoint + Kalman se ativo)
        if (follower != null) {
            follower.update();
            if (kalmanFilter != null) {
                kalmanFilter.update();
            }
        }

        // Drive (starting pose igual TeleOp; permite mover e ver distância mudar)
        if (fod != null) {
            fod.movement(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_bumper
            );
        }

        // Y: reset pose para starting pose
        boolean yNow = gamepad1.y;
        if (yNow && !yPrev && follower != null) {
            follower.setPose(START_POSE);
        }
        yPrev = yNow;

        // B: alternar modo shooter Manual <-> Kalman (distância)
        boolean bNow = gamepad1.b;
        if (bNow && !bPrev) {
            useDistanceBasedVelocity = !useDistanceBasedVelocity;
        }
        bPrev = bNow;

        // Intake toggle (left trigger) - igual ao TeleOp
        boolean leftTriggerNow = gamepad1.left_trigger > 0.1;
        if (leftTriggerNow && !leftTriggerPrev) {
            intakeActive = !intakeActive;
        }
        leftTriggerPrev = leftTriggerNow;

        double intakePower = intakeActive ? ConstantsConf.Intake.INTAKE_POWER : 0.0;
        if (intakeMotor != null) intakeMotor.setPower(intakePower);
        if (intakeMotor2 != null) intakeMotor2.setPower(intakePower);

        // Flap control (right trigger) - igual ao TeleOp
        boolean rightTriggerNow = gamepad1.right_trigger > 0.1;
        if (rightTriggerNow && !rightTriggerPrev && flapState == FlapState.NORMAL) {
            flapState = FlapState.ALIGNING;
            flapTimer.reset();
            setFlapPosition(FLAP_ALIGNED_POSITION);
        }
        rightTriggerPrev = rightTriggerNow;

        // Update flap state machine
        updateFlap();

        // Shooter velocity: Manual (D-Pad) ou Kalman (distância ao goal)
        double effectiveVelocity = curTargetVelocity;
        double distanceToGoal = 0.0;
        double rpmFromDistance = 0.0;
        String distanceError = null;

        if (follower != null) {
            Pose pose = follower.getPose();
            double dx = GOAL_X - pose.getX();
            double dy = GOAL_Y - pose.getY();
            distanceToGoal = Math.hypot(dx, dy);
            if (useDistanceBasedVelocity) {
                if (!Double.isFinite(distanceToGoal) || distanceToGoal < 1.0) {
                    distanceError = "Pose invalida ou distancia < 1 pol";
                    distanceToGoal = Math.max(1.0, ConstantsConf.Shooter.DIST_NEAR_POL);
                }
                rpmFromDistance = distanceToRPM.getRPM(distanceToGoal);
                effectiveVelocity = rpmToTicksPerSecond(rpmFromDistance);
            }
        }

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
                        scaleFactor = 10.0;
                        break;
                    case 1:
                        scaleFactor = 100.0;
                        break;
                    case 2:
                        scaleFactor = 1000.0;
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
                leftFlywheel.setVelocity(effectiveVelocity);
                rightFlywheel.setVelocity(effectiveVelocity);
            } else {
                leftFlywheel.setVelocity(0);
                rightFlywheel.setVelocity(0);
            }
        }

        // Telemetry
        telemetry.clear();

        // --- Modo e pose (para debug de erros) ---
        telemetry.addLine("=== MODO E POSE ===");
        telemetry.addData("Modo Shooter (B)", useDistanceBasedVelocity ? "KALMAN (distancia)" : "MANUAL (D-Pad)");
        if (follower != null) {
            Pose p = follower.getPose();
            telemetry.addData("Pose X (pol)", "%.2f", p.getX());
            telemetry.addData("Pose Y (pol)", "%.2f", p.getY());
            telemetry.addData("Heading (°)", "%.1f", Math.toDegrees(p.getHeading()));
            telemetry.addData("Goal", "(%.1f, %.1f)", GOAL_X, GOAL_Y);
            telemetry.addData("Distancia ao goal (pol)", "%.2f", distanceToGoal);
            if (distanceError != null) {
                telemetry.addData("ERRO distancia", distanceError);
            }
        } else {
            telemetry.addData("Pose", "Follower NAO disponivel");
        }

        if (kalmanFilter != null) {
            telemetry.addData("Fusao Limelight", kalmanFilter.isVisionFusionEnabled() ? "ON" : "OFF");
            telemetry.addData("Has Vision", kalmanFilter.hasVision() ? "SIM" : "NAO");
            if (!kalmanFilter.hasVision()) {
                telemetry.addData("Motivo rejeicao", kalmanFilter.getLastRejectionReason());
                telemetry.addData("Tags vistas", kalmanFilter.getLastFiducialIdsSeen());
            }
        }

        telemetry.addLine();
        telemetry.addLine("=== FLYWHEEL SHOOTER ===");
        if (leftFlywheel != null && rightFlywheel != null) {
            double curVelocityLeft = leftFlywheel.getVelocity();
            double curVelocityRight = rightFlywheel.getVelocity();
            double curVelocityAvg = (curVelocityLeft + curVelocityRight) / 2.0;
            double targetUsed = useDistanceBasedVelocity ? effectiveVelocity : curTargetVelocity;
            double error = targetUsed - curVelocityAvg;

            // Cálculo automático: ticks/s → RPM para target e currents
            double targetRpm = ticksPerSecondToRpm(targetUsed);
            double currentLeftRpm = ticksPerSecondToRpm(curVelocityLeft);
            double currentRightRpm = ticksPerSecondToRpm(curVelocityRight);
            double currentAvgRpm = ticksPerSecondToRpm(curVelocityAvg);

            telemetry.addData("Velocidade em uso", useDistanceBasedVelocity ? "por distancia" : "manual");
            telemetry.addData("Target (RPM)", "%.0f", targetRpm);
            if (useDistanceBasedVelocity) {
                telemetry.addData("RPM (da LUT)", "%.0f", rpmFromDistance);
            }
            telemetry.addData("Manual (D-Pad) guardado", "%.0f ticks/s", curTargetVelocity);
            telemetry.addData("Current Left (RPM)", "%.0f", currentLeftRpm);
            telemetry.addData("Current Right (RPM)", "%.0f", currentRightRpm);
            telemetry.addData("Current Avg (RPM)", "%.0f", currentAvgRpm);
            telemetry.addData("Error (ticks/s)", "%.0f", error);
            telemetry.addData("Active (A)", shooterActive ? "YES" : "NO");
            telemetry.addData("Scale Factor (X)", "%.0f", scaleFactor);
        } else {
            telemetry.addData("Status", "Motores NAO disponiveis");
        }

        telemetry.addLine();
        telemetry.addLine("=== INTAKE (intake + intake_2) ===");
        if (intakeMotor != null || intakeMotor2 != null) {
            telemetry.addData("Active (LT)", intakeActive ? "ON" : "OFF");
            telemetry.addData("Power", "%.2f", intakePower);
        } else {
            telemetry.addData("Status", "Motores NAO disponiveis");
        }

        telemetry.addLine();
        telemetry.addLine("=== FLAP SERVO(S) ===");
        if (flapServo != null || flapServo2 != null) {
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
            if (flapServo != null) telemetry.addData("Position", "%.2f", flapServo.getPosition());
            telemetry.addData("Control (RT)", "Pressione para ciclo");
        } else {
            telemetry.addData("Status", "Servos NAO disponiveis");
        }

        telemetry.addLine();
        telemetry.addLine("=== CONTROLES ===");
        telemetry.addData("LT", "Toggle intake");
        telemetry.addData("RT", "Flap cycle");
        telemetry.addData("D-Pad U/D", "Velocidade (Manual)");
        telemetry.addData("X", "Scale: " + scaleFactor);
        telemetry.addData("A", "Toggle shooter");
        telemetry.addData("B", "Modo: " + (useDistanceBasedVelocity ? "Kalman" : "Manual"));
        telemetry.addData("Y", "Reset pose");

        telemetry.update();
    }

    private double rpmToTicksPerSecond(double rpm) {
        if (ConstantsConf.Shooter.TICKS_PER_REVOLUTION <= 0) return 0;
        return rpm * ConstantsConf.Shooter.TICKS_PER_REVOLUTION / 60.0;
    }

    /** Converte ticks/s para RPM (cálculo automático). */
    private double ticksPerSecondToRpm(double ticksPerSecond) {
        if (ConstantsConf.Shooter.TICKS_PER_REVOLUTION <= 0) return 0;
        return ticksPerSecond * 60.0 / ConstantsConf.Shooter.TICKS_PER_REVOLUTION;
    }

    private void setFlapPosition(double position) {
        if (flapServo != null) flapServo.setPosition(position);
        if (flapServo2 != null) flapServo2.setPosition(position);
    }

    /**
     * Update flap state machine (igual ao IntakeSubsystem).
     */
    private void updateFlap() {
        if (flapServo == null && flapServo2 == null) return;

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

    @Override
    public void stop() {
        if (kalmanFilter != null) {
            kalmanFilter.stop();
        }
        setFlapPosition(FLAP_NORMAL_POSITION);
        if (intakeMotor != null) intakeMotor.setPower(0.0);
        if (intakeMotor2 != null) intakeMotor2.setPower(0.0);
        if (leftFlywheel != null) {
            leftFlywheel.setVelocity(0);
        }
        if (rightFlywheel != null) {
            rightFlywheel.setVelocity(0);
        }
    }
}
