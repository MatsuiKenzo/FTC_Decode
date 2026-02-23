package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.national.hardware.RobotHardwareNacional;
import org.firstinspires.ftc.teamcode.drive.national.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Tuner da interpolação linear distância → RPM do shooter (Nacional).
 *
 * Uso: posicione o robô em distâncias conhecidas (ex.: 63, 99, 145 pol), ajuste o RPM
 * (Stick Y GP2) até o tiro ficar bom, anote o par (distância, RPM) e adicione em
 * ConstantsConf.Shooter: DISTANCE_LUT_POL e RPM_LUT (ou DIST_*_POL / RPM_* para 3 pontos).
 *
 * Gamepad 1:
 *   - Stick: drive | LB: reset IMU | B: reset pose | Y: recalibrar alvo | A: toggle turret lock
 *   - LT: intake | RT: shoot (flap)
 *
 * Gamepad 2:
 *   - Stick Y: ajustar target RPM | A: aplicar RPM | B: reset RPM (1500)
 *
 * Se não tiver turret: use RobotHardwareNacional(hardwareMap, follower, false).
 */
@TeleOp(name = "Linear Interpolation Tuner", group = "Tuning")
public class LinearInterpolationTuner extends LinearOpMode {

    private RobotHardwareNacional robot;
    private FieldOrientedDrive fod;
    private Follower follower;

    private final Pose startPose = new Pose(39, 80, Math.toRadians(180));
    private static final double TARGET_X = 6.0;
    private static final double TARGET_Y = 138.0;

    private double targetRPM = 1500.0;

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

        // No nacional, passamos o hardwareMap e o follower
        robot = new RobotHardwareNacional(hardwareMap, follower);
        fod = new FieldOrientedDrive(hardwareMap);

        // Configuração inicial do Shooter Nacional
        robot.shooter.setTargetPosition(TARGET_X, TARGET_Y);
        robot.shooter.setUseDistanceBasedVelocity(false); // Desativa o automático para calibração manual

        telemetry.addData("Status", "Linear Interpolation Tuner (distância → RPM)");
        telemetry.addData("Info", "GP1=drive/intake, GP2=RPM. Anote (dist, RPM) → ConstantsConf.Shooter");
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

            // Intake toggle no LT (usando a lógica do nacional que espera um boolean)
            boolean leftTriggerNow = gamepad1.left_trigger > 0.1;
            robot.intake.toggleIntake(leftTriggerNow && !leftTriggerPrev);
            leftTriggerPrev = leftTriggerNow;

            // Shoot (Flap) no RT (usando a lógica do nacional que espera um boolean)
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

            // Gamepad 2: RPM
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
            telemetry.update();
        }

        robot.stop();
    }
}
