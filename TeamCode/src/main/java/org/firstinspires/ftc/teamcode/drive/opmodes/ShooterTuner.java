package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 *
 * Gamepad 1:
 *   - Stick: drive | LB: reset IMU | B: reset pose | Y: goal = onde está mirando | A: lock turret
 *   - LT + RT: coleta (intake) | RT: tiro (indexer para no sensor)
 *
 * Gamepad 2:
 *   - D-Pad Up/Down: kP | D-Pad Left/Right: kI | Bumpers: kD
 *   - Stick Y: ajustar target RPM | A: aplicar RPM no shooter | B: reset PID e RPM
 */
@TeleOp(name = "Shooter Tuner", group = "Tuning")
public class ShooterTuner extends LinearOpMode {

    private RobotHardware robot;
    private FieldOrientedDrive fod;
    private Follower follower;

    /** Pose inicial igual ao TeleOp Blue (ajustar se for Red). */
    private final Pose startPose = new Pose(39, 80, Math.toRadians(180));
    /** Alvo = gol (mesmo do TeleOpBlue). */
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

    @Override
    public void runOpMode() {
        fod = new FieldOrientedDrive(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        robot = new RobotHardware(hardwareMap, follower);
        robot.setTargetPosition(TARGET_X, TARGET_Y);
        // Shooter obedece só ao RPM do GP2, não ao cálculo por distância (odometria continua para você anotar distância)
        robot.shooter.setUseDistanceBasedVelocity(false);

        telemetry.addData("Status", "Inicializado. GP1=drive+intake+tiro (igual TeleOp), GP2=ajustes RPM/PID.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            robot.update();

            // Haptic quando shooter pronto
            boolean shooterReady = robot.shooter.isReady();
            if (shooterReady && !shooterWasReady) {
                gamepad1.rumble(1.0, 1.0, 250);
            }
            shooterWasReady = shooterReady;

            //Gamepad 1: movimentação, intake e tiro
            fod.movement(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_bumper
            );
            robot.intake.collect(-gamepad1.left_trigger, -gamepad1.right_trigger);
            robot.intake.shoot(gamepad1.right_trigger);
            if (gamepad1.b) {
                follower.setPose(startPose);
            }
            // Recalibrar goal = onde está mirando (Y) — só o ângulo
            boolean yNow = gamepad1.y;
            if (yNow && !yPrevGp1) {
                Pose robotPose = follower.getPose();
                double dist = robot.shooter.getDistance();
                double headingRad = robotPose.getHeading();
                double turretDeg = robot.turret.getMotorAngle();
                double absoluteAngleRad = headingRad + Math.toRadians(turretDeg);
                double newTargetX = robotPose.getX() + dist * Math.cos(absoluteAngleRad);
                double newTargetY = robotPose.getY() + dist * Math.sin(absoluteAngleRad);
                robot.setTargetPosition(newTargetX, newTargetY);
            }
            yPrevGp1 = yNow;
            // Turret lock: toggle com A (um clique trava, outro destrava)
            boolean aNow = gamepad1.a;
            if (aNow && !aPrevGp1) {
                turretLocked = !turretLocked;
            }
            aPrevGp1 = aNow;
            if (turretLocked) {
                robot.turret.lockAngle(-45.0);
            } else {
                robot.turret.unlockAngle();
            }

            // Gamepad 2: só ajustes de potência/PID e RPM
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
            if (Math.abs(gamepad2.right_stick_y) > 0.1) {
                targetRPM += gamepad2.right_stick_y * 50;
                targetRPM = Math.max(0, Math.min(6000, targetRPM));
            }
            robot.shooter.setPID(kP, kI, kD, kF);
            if (gamepad2.a) {
                robot.shooter.setTargetRPM(targetRPM);
            }
            if (gamepad2.b) {
                kP = 0.01;
                kI = 0.0001;
                kD = 0.0001;
                kF = 0.0001;
                robot.shooter.setPID(kP, kI, kD, kF);
                targetRPM = 1500.0;
                robot.shooter.setTargetRPM(targetRPM);
            }

            //Telemetria: odometria + distância + RPM para calibração
            double distPol = robot.shooter.getDistance();
            telemetry.addData("--- Odometria (posicione com GP1) ---", "");
            telemetry.addData("Pose X (pol)", "%.1f", follower.getPose().getX());
            telemetry.addData("Pose Y (pol)", "%.1f", follower.getPose().getY());
            telemetry.addData("Heading (°)", "%.1f", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Distância ao gol (pol)", "%.1f", distPol);
            telemetry.addLine();
            telemetry.addData(">>> Calibração: use este valor <<<", "");
            telemetry.addData("Distância (pol)", "%.1f", distPol);
            telemetry.addData("RPM neste ponto", "%.0f", targetRPM);
            telemetry.addData("(Anote: perto/meio/longe = X pol → Y RPM)", "");
            telemetry.addLine();
            telemetry.addData("--- Shooter ---", "");
            telemetry.addData("Target RPM", "%.0f", targetRPM);
            telemetry.addData("Current RPM", "%.0f", robot.shooter.getCurrentRPM());
            telemetry.addData("Ready", robot.shooter.isReady() ? "YES" : "NO");
            telemetry.addLine();
            telemetry.addData("--- Intake ---", "");
            telemetry.addData("Has Ball", robot.intake.hasBall() ? "YES" : "NO");
            telemetry.addData("Ball Color", robot.intake.getBallColor());
            telemetry.update();
        }

        fod.movement(0, 0, 0, false);
        robot.stop();
    }
}
