package org.firstinspires.ftc.teamcode.drive.national.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.national.actuators.KalmanFilterLocalizer;
import org.firstinspires.ftc.teamcode.drive.national.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.drive.national.hardware.RobotHardwareNacional;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * TeleOp Blue Nacional — SEM TURRET (para teste sem servos da turret conectados).
 *
 * Igual ao TeleOp Blue Nacional, porém sem inicializar nem usar a turret.
 * Use este OpMode para testar o robô enquanto os servos da turret não estão conectados.
 *
 * Features: drive field-oriented, shooter (2 motores), hood opcional, intake, Kalman/Limelight.
 * Controls: B = reset pose, Y = recalibrar goal (direção do robô), X = toggle fusão Limelight.
 */
@TeleOp(name = "TeleOp Blue Nacional (Sem Turret)", group = "Nacional")
public class TeleOpBlueNacionalSemTurret extends OpMode {
    private FieldOrientedDrive fod;
    private RobotHardwareNacional robot;
    private Follower follower;
    private KalmanFilterLocalizer kalmanFilter;
    private boolean shooterWasReady = false;
    private boolean yPrev = false;
    private boolean xPrev = false;
    private boolean leftTriggerPrev = false;
    private boolean rightTriggerPrev = false;

    private final Pose startTeleop = new Pose(39, 80, Math.toRadians(180));
    private double targetX = 6.0;
    private double targetY = 140.0;

    @Override
    public void init() {
        fod = new FieldOrientedDrive(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startTeleop);

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

        // Sem turret — para teste sem servos conectados
        robot = new RobotHardwareNacional(hardwareMap, follower, false);
        robot.setTargetPosition(targetX, targetY);

        if (robot.hood != null && robot.hood.isEnabled()) {
            telemetry.addData("Hood/Tilt", "ATIVO");
        } else {
            telemetry.addData("Hood/Tilt", "DESATIVADO (servo nao conectado ou TILT_ENABLED=false)");
        }

        telemetry.addData("Status", "Inicializado. Nacional SEM TURRET (teste).");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        if (kalmanFilter != null) {
            kalmanFilter.update();
        }
        robot.update();

        boolean shooterReady = robot.shooter.isReady();
        if (shooterReady && !shooterWasReady) {
            gamepad1.rumble(1.0, 1.0, 250);
        }
        shooterWasReady = shooterReady;

        fod.movement(
                -gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                gamepad1.left_bumper
        );

        boolean leftTriggerNow = gamepad1.left_trigger > 0.1;
        if (leftTriggerNow && !leftTriggerPrev) {
            robot.intake.toggleIntake(true);
        } else {
            robot.intake.toggleIntake(false);
        }
        leftTriggerPrev = leftTriggerNow;

        boolean rightTriggerNow = gamepad1.right_trigger > 0.1;
        if (rightTriggerNow && !rightTriggerPrev) {
            robot.intake.shoot(true);
        } else {
            robot.intake.shoot(false);
        }
        rightTriggerPrev = rightTriggerNow;

        if (gamepad1.b) {
            follower.setPose(startTeleop);
        }

        // Recalibrar goal (Y) — direção do robô (sem turret)
        boolean yNow = gamepad1.y;
        if (yNow && !yPrev) {
            Pose robotPose = follower.getPose();
            double dist = robot.shooter.getDistance();
            double headingRad = robotPose.getHeading();
            double newTargetX = robotPose.getX() + dist * Math.cos(headingRad);
            double newTargetY = robotPose.getY() + dist * Math.sin(headingRad);
            robot.setTargetPosition(newTargetX, newTargetY);
        }
        yPrev = yNow;

        boolean xNow = gamepad1.x;
        if (xNow && !xPrev && kalmanFilter != null) {
            kalmanFilter.setVisionFusionEnabled(!kalmanFilter.isVisionFusionEnabled());
        }
        xPrev = xNow;

        telemetry.addData("--- Shooter Nacional (2 motores) ---", "");
        telemetry.addData("Target Velocity", "%.0f", robot.shooter.getTargetVelocity());
        telemetry.addData("Current Velocity (L)", "%.0f", robot.shooter.getCurrentVelocityLeft());
        telemetry.addData("Current Velocity (R)", "%.0f", robot.shooter.getCurrentVelocityRight());
        telemetry.addData("Current Velocity (Avg)", "%.0f", robot.shooter.getCurrentVelocity());
        telemetry.addData("Ready", robot.shooter.isReady() ? "YES ✓" : "NO ✗");
        telemetry.addData("Distance", "%.1f pol", robot.shooter.getDistance());
        telemetry.addData("Battery", "%.2f V", robot.shooter.getVoltage());
        telemetry.addData("Y", "Recalibrar goal = onde está mirando");

        if (robot.hood != null && robot.hood.isEnabled()) {
            telemetry.addData("--- Hood/Tilt ---", "");
            telemetry.addData("Position", "%.2f", robot.hood.getCurrentAngle());
            telemetry.addData("Distance", "%.1f pol", robot.hood.getDistance());
        }

        telemetry.addData("--- Pose ---", "");
        telemetry.addData("X", "%.1f", follower.getPose().getX());
        telemetry.addData("Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        if (kalmanFilter != null) {
            telemetry.addData("Fusao (X)", kalmanFilter.isVisionFusionEnabled() ? "ON" : "OFF");
            if (kalmanFilter.hasVision()) {
                telemetry.addData("Visao", "Tags: %d", kalmanFilter.getValidTagCount());
            }
        }

        telemetry.addData("--- Intake ---", "");
        telemetry.addData("Intake Active", robot.intake.isIntakeActive() ? "ON" : "OFF");

        telemetry.update();
    }

    @Override
    public void stop() {
        if (kalmanFilter != null) {
            kalmanFilter.stop();
        }
        robot.stop();
    }
}
