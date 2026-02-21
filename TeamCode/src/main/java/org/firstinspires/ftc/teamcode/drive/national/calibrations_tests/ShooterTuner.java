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
 * Shooter Tuner adaptado para o hardware NACIONAL.
 * Mantém exatamente a mesma lógica de controle do código original.
 *
 * Gamepad 1:
 *   - Stick: drive | LB: reset IMU | B: reset pose | Y: recalibrar alvo | A: toggle turret lock
 *   - LT: toggle intake | RT: shoot (flap)
 *
 * Gamepad 2:
 *   - D-Pad Up/Down: kP | D-Pad Left/Right: kI | Bumpers: kD
 *   - Stick Y: ajustar target RPM | A: aplicar RPM | B: reset PID e RPM
 *
 * Se não tiver turret conectada: use robot = new RobotHardwareNacional(hardwareMap, follower, false);
 * assim o Y recalibra só pela direção do robô e o A é ignorado.
 */
@TeleOp(name = "Shooter Tuner Nacional", group = "Tuning")
public class ShooterTuner extends LinearOpMode {

    private RobotHardwareNacional robot;
    private FieldOrientedDrive fod;
    private Follower follower;

    // Pose inicial e Alvo (mantidos do original)
    private final Pose startPose = new Pose(39, 80, Math.toRadians(180));
    private static final double TARGET_X = 6.0;
    private static final double TARGET_Y = 138.0;

    private double kP = 0.01;
    private double kI = 0.0001;
    private double kD = 0.0001;
    private double kF = 0.0001;
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

        telemetry.addData("Status", "Inicializado - HARDWARE NACIONAL");
        telemetry.addData("Info", "GP1=drive/intake, GP2=ajustes RPM/PID");
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

            // Gamepad 2: Ajustes de PID e RPM
            if (gamepad2.dpad_up) {
                kP += 0.001;
                sleep(200);
            } else if (gamepad2.dpad_down) {
                kP -= 0.001;
                sleep(200);
            }
            if (gamepad2.dpad_left) {
                kI += 0.00001;
                sleep(200);
            } else if (gamepad2.dpad_right) {
                kI -= 0.00001;
                sleep(200);
            }
            if (gamepad2.left_bumper) {
                kD += 0.00001;
                sleep(200);
            } else if (gamepad2.right_bumper) {
                kD -= 0.00001;
                sleep(200);
            }

            // Ajuste de RPM no Stick Y
            if (Math.abs(gamepad2.right_stick_y) > 0.1) {
                targetRPM -= gamepad2.right_stick_y * 50; // Invertido para stick up = aumentar
                targetRPM = Math.max(0, Math.min(6000, targetRPM));
            }

            // Aplica PID ao Shooter Nacional (afeta ambos os motores)
            robot.shooter.setPID(kP, kI, kD, kF);

            if (gamepad2.a) {
                robot.shooter.setTargetRPM(targetRPM);
            }

            // Reset de calibração no B
            if (gamepad2.b) {
                kP = 0.01; kI = 0.0001; kD = 0.0001; kF = 0.0001;
                robot.shooter.setPID(kP, kI, kD, kF);
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
            telemetry.addData(">>> CALIBRAÇÃO <<<", "");
            telemetry.addData("Distância ao Alvo (pol)", "%.1f", distPol);
            telemetry.addData("RPM Alvo", "%.0f", targetRPM);
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
