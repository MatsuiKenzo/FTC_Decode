package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.national.hardware.RobotHardwareNacional;
import org.firstinspires.ftc.teamcode.drive.national.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Tuner da interpolação linear distância → RPM do shooter (Nacional).
 * Intake e flap como no Flap Intake Tester: LT = toggle intake, RT = flap (alinhar → 2s → voltar).
 * Potência do intake ajustável no Gamepad 2 (stick esquerdo Y) para calibrar melhor.
 *
 * Gamepad 1:
 *   - Stick: drive | LB: reset IMU | B: reset pose | Y: recalibrar alvo | A: toggle turret lock
 *   - LT: toggle intake | RT: shoot (flap)
 *
 * Gamepad 2:
 *   - Stick esquerdo Y: potência do intake (0 a 1)
 *   - Stick direito Y: ajustar target RPM | A: aplicar RPM | B: reset RPM (1500)
 *
 * Se não tiver turret: use RobotHardwareNacional(hardwareMap, follower, false).
 */
@TeleOp(name = "Linear Interpolation Tuner", group = "Tuning")
public class LinearInterpolationTuner extends LinearOpMode {

    private RobotHardwareNacional robot;
    private FieldOrientedDrive fod;
    private Follower follower;

    /** Motor opcional intake_2 (igual Flap Intake Tester); mesma potência do intake quando ativo. */
    private DcMotorEx intakeMotor2;

    private final Pose startPose = new Pose(39, 80, Math.toRadians(180));
    private static final double TARGET_X = 6.0;
    private static final double TARGET_Y = 138.0;

    private double targetRPM = 1500.0;
    /** Potência do intake (0 a 1), ajustável no GP2 left stick Y. */
    private double intakePower = ConstantsConf.Intake.INTAKE_POWER;

    private boolean shooterWasReady = false;
    private boolean turretLocked = false;
    private boolean aPrevGp1 = false;
    private boolean yPrevGp1 = false;
    private boolean leftTriggerPrev = false;
    private boolean rightTriggerPrev = false;

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

        // Configuração inicial do Shooter Nacional
        robot.shooter.setTargetPosition(TARGET_X, TARGET_Y);
        robot.shooter.setUseDistanceBasedVelocity(false); // Desativa o automático para calibração manual

        telemetry.addData("Status", "Linear Interpolation Tuner (distância → RPM)");
        telemetry.addData("Info", "GP1=drive, LT=intake RT=flap | GP2=potência intake (L stick) + RPM (R stick)");
        telemetry.update();

        waitForStart();

        // Inicia o flywheel no RPM de teste para calibração
        robot.shooter.setTargetRPM(targetRPM);

        while (opModeIsActive()) {
            follower.update();
            // O robot.update() no nacional gerencia o shooter, turret e hood
            robot.update();

            // Haptic feedback quando o shooter nacional estiver pronto
            boolean shooterReady = robot.shooter.isReady();
            if (shooterReady && !shooterWasReady) {
                gamepad1.rumble(1.0, 1.0, 250);
            }
            shooterWasReady = shooterReady;

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
                double headingRad = robotPose.getHeading();
                double absoluteAngleRad = headingRad;
                if (robot.turret != null) {
                    absoluteAngleRad = headingRad + Math.toRadians(robot.turret.getMotorAngle());
                }
                double newTargetX = robotPose.getX() + dist * Math.cos(absoluteAngleRad);
                double newTargetY = robotPose.getY() + dist * Math.sin(absoluteAngleRad);
                robot.shooter.setTargetPosition(newTargetX, newTargetY);
            }
            yPrevGp1 = yNow;

            // Turret Lock no A (só se turret estiver conectada)
            boolean aNow = gamepad1.a;
            if (aNow && !aPrevGp1) {
                turretLocked = !turretLocked;
            }
            aPrevGp1 = aNow;
            if (robot.turret != null) {
                if (turretLocked) {
                    robot.turret.lockAngle(-45.0);
                } else {
                    robot.turret.unlockAngle();
                }
            }

            // Gamepad 2: RPM (stick direito)
            if (Math.abs(gamepad2.right_stick_y) > 0.1) {
                targetRPM -= gamepad2.right_stick_y * 50;
                targetRPM = Math.max(0, Math.min(6000, targetRPM));
            }
            if (gamepad2.a) {
                robot.shooter.setTargetRPM(targetRPM);
            }
            if (gamepad2.b) {
                targetRPM = 1500.0;
                robot.shooter.setTargetRPM(targetRPM);
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
            telemetry.addData("RPM Alvo", "%.0f", targetRPM);
            telemetry.addData("Ponto para LUT", "%.1f pol, %.0f RPM → ConstantsConf.Shooter", distPol, targetRPM);
            telemetry.addLine();
            // getCurrentVelocityLeft/Right retornam ticks/s; converter para RPM para exibição
            double tpr = ConstantsConf.Shooter.TICKS_PER_REVOLUTION;
            double rpmLeft = tpr > 0 ? robot.shooter.getCurrentVelocityLeft() / tpr * 60.0 : 0;
            double rpmRight = tpr > 0 ? robot.shooter.getCurrentVelocityRight() / tpr * 60.0 : 0;
            telemetry.addData("--- Shooter Nacional (2 Motores) ---", "");
            telemetry.addData("RPM Left", "%.0f", rpmLeft);
            telemetry.addData("RPM Right", "%.0f", rpmRight);
            telemetry.addData("RPM Média", "%.0f", robot.shooter.getCurrentRPM());
            telemetry.addData("Ready", robot.shooter.isReady() ? "SIM" : "NÃO");
            telemetry.addLine();
            telemetry.addData("--- Intake (igual Flap Intake Tester) ---", "");
            telemetry.addData("Intake (LT)", robot.intake.isIntakeActive() ? "ON" : "OFF");
            telemetry.addData("Potência intake (GP2 L stick Y)", "%.2f", intakePower);
            telemetry.addData("Intake_2", intakeMotor2 != null ? "conectado" : "não configurado");
            telemetry.addLine();
            telemetry.addData("Flap (RT)", "alinhar → 2s → voltar");
            telemetry.update();
        }

        if (intakeMotor2 != null) intakeMotor2.setPower(0.0);
        robot.stop();
    }
}
