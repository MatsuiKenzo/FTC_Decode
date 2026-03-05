package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.national.hardware.RobotHardwareNacional;
import org.firstinspires.ftc.teamcode.drive.national.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Tuner da interpolação linear distância → RPM.
 * A = ligar/desligar shooter no RPM definido. D-Pad Up/Down = ajustar RPM, X = escala (50/100/500 RPM).
 * Intake e flap como Flap Intake Tester: LT = intake, RT = flap.
 *
 * Gamepad 1:
 *   - Stick: drive | LB: reset IMU | B: reset pose | Y: recalibrar alvo
 *   - LT: toggle intake | RT: flap | A: toggle shooter | D-Pad U/D: RPM | X: escala (RPM por pressão)
 *
 * Gamepad 2:
 *   - Stick esquerdo Y: potência do intake (0 a 1)
 *   - D-Pad Left: ciclar hood (índice 0 → 1 → 2 → 0)
 *   - D-Pad Up/Down: ajustar valor da posição atual do hood (copie para ConstantsConf.Nacional.HOOD_CYCLE_POSITION_*)
 */
@TeleOp(name = "Linear Interpolation Tuner", group = "Tuning")
public class LinearInterpolationTuner extends LinearOpMode {

    private RobotHardwareNacional robot;
    private FieldOrientedDrive fod;
    private Follower follower;

    /** Shooter: controle direto dos motores para não engasgar e zero parar. */
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;

    /** Motor opcional intake_2. */
    private DcMotorEx intakeMotor2;

    private final Pose startPose = new Pose(39, 80, Math.toRadians(180));
    private static final double TARGET_X = 6.0;
    private static final double TARGET_Y = 138.0;

    /** RPM alvo. A = liga/desliga nesse RPM. */
    private double curTargetRPM = 2500.0;
    private double scaleFactorRPM = 50.0;
    private int scaleMode = 0; // 0=50, 1=100, 2=500 RPM por pressão
    private boolean shooterActive = false;

    private double intakePower = ConstantsConf.Intake.INTAKE_POWER;

    /** Hood: 3 posições cicláveis (ajustáveis no tuner). Inicializadas de ConstantsConf. */
    private final double[] hoodCyclePositions = new double[3];
    private int hoodCycleIndex = 0;

    private boolean aPrevGp1 = false;
    private boolean yPrevGp1 = false;
    private boolean xPrevGp1 = false;
    private boolean leftTriggerPrev = false;
    private boolean rightTriggerPrev = false;
    private boolean dpadLeft2Prev = false;
    private boolean dpadUp2Prev = false;
    private boolean dpadDown2Prev = false;

    @Override
    public void runOpMode() {
        // Inicialização do PedroPathing e Hardware Nacional
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        // Se NÃO tiver turret conectada, use: new RobotHardwareNacional(hardwareMap, follower, false)
        robot = new RobotHardwareNacional(hardwareMap, follower, false);
        fod = new FieldOrientedDrive(hardwareMap);

        // Intake_2 opcional (igual Flap Intake Tester)
        try {
            intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake_2");
            intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            intakeMotor2 = null;
        }

        // Shooter: motores diretos como nas classes de calibração
        try {
            leftFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_LEFT_MOTOR_NAME);
            rightFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_RIGHT_MOTOR_NAME);
            leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
            PIDFCoefficients pidf = new PIDFCoefficients(
                    ConstantsConf.Shooter.KP,
                    ConstantsConf.Shooter.KI,
                    ConstantsConf.Shooter.KD,
                    ConstantsConf.Shooter.KF
            );
            leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        } catch (Exception e) {
            leftFlywheel = null;
            rightFlywheel = null;
        }

        hoodCyclePositions[0] = ConstantsConf.Nacional.HOOD_CYCLE_POSITION_0;
        hoodCyclePositions[1] = ConstantsConf.Nacional.HOOD_CYCLE_POSITION_1;
        hoodCyclePositions[2] = ConstantsConf.Nacional.HOOD_CYCLE_POSITION_2;

        telemetry.addData("Status", "Linear Interpolation Tuner (distância → RPM)");
        telemetry.addData("Info", "GP1: A=shooter D-Pad=RPM X=escala LT=intake RT=flap | GP2: L stick=intake, D-Pad L=ciclar hood U/D=ajustar");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            robot.intake.update(); // só flap state machine; shooter é controle direto abaixo

            // Hood: aplicar posição ciclável (ajustável no tuner)
            if (robot.hood != null && robot.hood.isEnabled()) {
                robot.hood.setPositionOverride(hoodCyclePositions[hoodCycleIndex]);
            }
            robot.updateWithoutShooter();

            // Gamepad 1: Movimentação (Field Oriented)
            fod.movement(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_bumper
            );

            // Intake toggle no LT (igual Flap Intake Tester)
            boolean leftTriggerNow = gamepad1.left_trigger > 0.1;
            robot.intake.toggleIntake(leftTriggerNow && !leftTriggerPrev);
            leftTriggerPrev = leftTriggerNow;

            // Hood: GP2 D-Pad Left = ciclar; GP2 D-Pad Up/Down = ajustar valor atual
            if (robot.hood != null && robot.hood.isEnabled()) {
                boolean dpadLeft2Now = gamepad2.dpad_left;
                if (dpadLeft2Now && !dpadLeft2Prev) {
                    hoodCycleIndex = (hoodCycleIndex + 1) % 3;
                }
                dpadLeft2Prev = dpadLeft2Now;
                boolean dpadUp2Now = gamepad2.dpad_up;
                if (dpadUp2Now && !dpadUp2Prev) {
                    hoodCyclePositions[hoodCycleIndex] = Math.min(ConstantsConf.Nacional.HOOD_MANUAL_MAX, hoodCyclePositions[hoodCycleIndex] + ConstantsConf.Nacional.HOOD_MANUAL_ADJUST_STEP);
                }
                dpadUp2Prev = dpadUp2Now;
                boolean dpadDown2Now = gamepad2.dpad_down;
                if (dpadDown2Now && !dpadDown2Prev) {
                    hoodCyclePositions[hoodCycleIndex] = Math.max(ConstantsConf.Nacional.HOOD_MANUAL_MIN, hoodCyclePositions[hoodCycleIndex] - ConstantsConf.Nacional.HOOD_MANUAL_ADJUST_STEP);
                }
                dpadDown2Prev = dpadDown2Now;
            }

            // Potência do intake no GP2 stick esquerdo Y (0 a 1)
            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                intakePower += gamepad2.left_stick_y * 0.02;
                intakePower = Math.max(0.0, Math.min(1.0, intakePower));
            }
            if (robot.intake.isIntakeActive()) {
                robot.intake.setPower(intakePower);
                if (intakeMotor2 != null) intakeMotor2.setPower(intakePower);
            } else {
                if (intakeMotor2 != null) intakeMotor2.setPower(0.0);
            }

            // Shoot (Flap) no RT — ciclo alinhar → 2s → voltar (igual Flap Intake Tester)
            boolean rightTriggerNow = gamepad1.right_trigger > 0.1;
            robot.intake.shoot(rightTriggerNow && !rightTriggerPrev);
            rightTriggerPrev = rightTriggerNow;

            // Reset Pose no B
            if (gamepad1.b) {
                follower.setPose(startPose);
            }

            // Recalibrar alvo no Y (direção = heading + turret se tiver turret, senão só heading)
            boolean yNow = gamepad1.y;
            if (yNow && !yPrevGp1) {
                Pose robotPose = follower.getPose();
                double dx = TARGET_X - robotPose.getX();
                double dy = TARGET_Y - robotPose.getY();
                double dist = Math.hypot(dx, dy);
                double headingRad = robotPose.getHeading() + Math.toRadians(ConstantsConf.Nacional.DRIVE_HEADING_OFFSET_DEG);
                double absoluteAngleRad = headingRad;
                if (robot.turret != null) {
                    absoluteAngleRad = headingRad + Math.toRadians(robot.turret.getMotorAngle());
                }
                double newTargetX = robotPose.getX() + dist * Math.cos(absoluteAngleRad);
                double newTargetY = robotPose.getY() + dist * Math.sin(absoluteAngleRad);
                robot.shooter.setTargetPosition(newTargetX, newTargetY);
            }
            yPrevGp1 = yNow;

            // Shooter: A = ligar/desligar; D-Pad Up/Down = RPM; X = escala (RPM por pressão)
            if (leftFlywheel != null && rightFlywheel != null) {
                if (gamepad1.dpad_up) {
                    curTargetRPM += scaleFactorRPM;
                    curTargetRPM = Math.min(ConstantsConf.Shooter.MAX_RPM, curTargetRPM);
                }
                if (gamepad1.dpad_down) {
                    curTargetRPM -= scaleFactorRPM;
                    curTargetRPM = Math.max(0, curTargetRPM);
                }
                boolean xNow = gamepad1.x;
                if (xNow && !xPrevGp1) {
                    scaleMode = (scaleMode + 1) % 3;
                    switch (scaleMode) {
                        case 0: scaleFactorRPM = 50.0; break;
                        case 1: scaleFactorRPM = 100.0; break;
                        case 2: scaleFactorRPM = 500.0; break;
                    }
                }
                xPrevGp1 = xNow;

                boolean aNow = gamepad1.a;
                if (aNow && !aPrevGp1) {
                    shooterActive = !shooterActive;
                }
                aPrevGp1 = aNow;

                double tpr = ConstantsConf.Shooter.TICKS_PER_REVOLUTION;
                double velocityTicksPerSec = tpr > 0 ? curTargetRPM * tpr / 60.0 : 0;
                if (shooterActive) {
                    leftFlywheel.setVelocity(velocityTicksPerSec);
                    rightFlywheel.setVelocity(velocityTicksPerSec);
                } else {
                    leftFlywheel.setVelocity(0);
                    rightFlywheel.setVelocity(0);
                }
            }

            // Telemetria para calibração
            Pose currentPose = follower.getPose();
            double dx = TARGET_X - currentPose.getX();
            double dy = TARGET_Y - currentPose.getY();
            double distPol = Math.hypot(dx, dy);

            telemetry.addData("--- Odometria (Nacional) ---", "");
            telemetry.addData("Pose X | Y", "%.1f | %.1f", currentPose.getX(), currentPose.getY());
            telemetry.addData("Heading (°)", "%.1f", Math.toDegrees(currentPose.getHeading()));
            telemetry.addLine();
            telemetry.addData(">>> INTERPOLAÇÃO (distância → RPM) <<<", "");
            telemetry.addData("Distância ao Alvo (pol)", "%.1f", distPol);
            telemetry.addData("RPM Alvo (D-Pad)", "%.0f", curTargetRPM);
            telemetry.addData("Ponto para LUT", "%.1f pol, %.0f RPM → ConstantsConf.Shooter", distPol, curTargetRPM);
            telemetry.addLine();
            telemetry.addData("--- Shooter (A=liga) ---", "");
            telemetry.addData("Shooter (A)", shooterActive ? "LIGADO" : "DESLIGADO");
            telemetry.addData("Target RPM", "%.0f  Escala (X)=%.0f RPM", curTargetRPM, scaleFactorRPM);
            if (leftFlywheel != null && rightFlywheel != null) {
                double tpr = ConstantsConf.Shooter.TICKS_PER_REVOLUTION;
                double vL = leftFlywheel.getVelocity();
                double vR = rightFlywheel.getVelocity();
                double rpmL = tpr > 0 ? vL / tpr * 60.0 : 0;
                double rpmR = tpr > 0 ? vR / tpr * 60.0 : 0;
                telemetry.addData("Current L | R (RPM)", "%.0f | %.0f", rpmL, rpmR);
            }
            telemetry.addLine();
            telemetry.addData("--- Intake (igual Flap Intake Tester) ---", "");
            telemetry.addData("Intake (LT)", robot.intake.isIntakeActive() ? "ON" : "OFF");
            telemetry.addData("Potência intake (GP2 L stick Y)", "%.2f", intakePower);
            telemetry.addData("Intake_2", intakeMotor2 != null ? "conectado" : "não configurado");
            telemetry.addLine();
            telemetry.addData("Flap (RT)", "alinhar → 2s → voltar");
            if (robot.hood != null && robot.hood.isEnabled()) {
                telemetry.addLine();
                telemetry.addData("--- Hood (GP2 D-Pad L=ciclar U/D=ajustar pouco a pouco) ---", "");
                telemetry.addData("Valor aplicado agora", "%.3f", hoodCyclePositions[hoodCycleIndex]);
                telemetry.addData("Posições", "0=%.3f  1=%.3f  2=%.3f  (índice=%d)", hoodCyclePositions[0], hoodCyclePositions[1], hoodCyclePositions[2], hoodCycleIndex);
                telemetry.addData("Passo (ConstantsConf)", "%.3f", ConstantsConf.Nacional.HOOD_MANUAL_ADJUST_STEP);
                telemetry.addData("Copie para ConstantsConf", "HOOD_CYCLE_POSITION_0/1/2");
            }
            telemetry.update();
        }

        if (intakeMotor2 != null) intakeMotor2.setPower(0.0);
        if (leftFlywheel != null) leftFlywheel.setVelocity(0);
        if (rightFlywheel != null) rightFlywheel.setVelocity(0);
        robot.stop();
    }
}
