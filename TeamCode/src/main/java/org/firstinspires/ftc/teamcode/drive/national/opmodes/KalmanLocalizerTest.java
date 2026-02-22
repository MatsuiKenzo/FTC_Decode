package org.firstinspires.ftc.teamcode.drive.national.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.national.actuators.KalmanFilterLocalizer;
import org.firstinspires.ftc.teamcode.drive.national.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * OpMode para testar o KalmanFilterLocalizer (Pinpoint + Limelight).
 * Permite pilotar o robô e ver na telemetria a pose atual, dados da Limelight e estado da fusão.
 *
 * Controles:
 * - Left stick: movimento
 * - Right stick X: giro
 * - Left bumper: reset IMU
 * - B: reset pose (volta para pose inicial)
 * - X: liga/desliga fusão Limelight (toggle)
 */

@TeleOp(name = "Test: Kalman Localizer", group = "Test")
public class KalmanLocalizerTest extends OpMode {
    private FieldOrientedDrive fod;
    private Follower follower;
    private KalmanFilterLocalizer kalmanFilter;

    private boolean xPrev = false;

    private static final Pose START_POSE = new Pose(39, 80, Math.toRadians(180));

    private static final double BLUE_GOAL_X = 7.0;
    private static final double BLUE_GOAL_Y = 107.0;
    private static final double RED_GOAL_X = 137.0;
    private static final double RED_GOAL_Y = 107.0;

    @Override
    public void init() {
        fod = new FieldOrientedDrive(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(START_POSE);

        kalmanFilter = new KalmanFilterLocalizer();
        if (kalmanFilter.init(hardwareMap, follower)) {
            telemetry.addData("Kalman", "Limelight + Pinpoint OK (tags 20, 24)");
        } else {
            telemetry.addData("Kalman", "Limelight nao encontrada - so Pinpoint");
        }
        telemetry.addData(">", "Pilote e veja a telemetria. B=reset pose, X=toggle fusao");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        if (kalmanFilter != null) {
            kalmanFilter.update();
        }

        fod.movement(
                -gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                gamepad1.left_bumper
        );

        if (gamepad1.b) {
            follower.setPose(START_POSE);
        }

        boolean xNow = gamepad1.x;
        if (xNow && !xPrev && kalmanFilter != null) {
            kalmanFilter.setVisionFusionEnabled(!kalmanFilter.isVisionFusionEnabled());
        }
        xPrev = xNow;

        // --- Telemetria ---
        Pose current = follower.getPose();
        telemetry.addData("--- Pose atual (em uso) ---", "");
        telemetry.addData("X (pol)", "%.2f", current.getX());
        telemetry.addData("Y (pol)", "%.2f", current.getY());
        telemetry.addData("Heading (°)", "%.1f", Math.toDegrees(current.getHeading()));

        double distBlue = Math.hypot(BLUE_GOAL_X - current.getX(), BLUE_GOAL_Y - current.getY());
        double distRed = Math.hypot(RED_GOAL_X - current.getX(), RED_GOAL_Y - current.getY());
        telemetry.addData("--- Distancia ao goal (pol) ---", "");
        telemetry.addData("Blue goal", "%.1f", distBlue);
        telemetry.addData("Red goal", "%.1f", distRed);

        if (kalmanFilter != null) {
            telemetry.addData("--- Fusao ---", "");
            telemetry.addData("Fusao Limelight (X)", kalmanFilter.isVisionFusionEnabled() ? "ON" : "OFF");

            if (kalmanFilter.hasVision()) {
                Pose ll = kalmanFilter.getLimelightPose();
                telemetry.addData("--- Limelight ---", "OK");
                telemetry.addData("LL X (pol)", "%.2f", ll.getX());
                telemetry.addData("LL Y (pol)", "%.2f", ll.getY());
                telemetry.addData("LL Heading (°)", "%.1f", Math.toDegrees(ll.getHeading()));
                telemetry.addData("Tags 20/24 visiveis", "%d", kalmanFilter.getValidTagCount());
                telemetry.addData("--- LL raw (metros) ---", "");
                telemetry.addData("LL X (m)", "%.3f", kalmanFilter.getLastVisionXMeters());
                telemetry.addData("LL Y (m)", "%.3f", kalmanFilter.getLastVisionYMeters());
                telemetry.addData("LL Yaw (°)", "%.1f", kalmanFilter.getLastVisionYawDeg());
                telemetry.addData("(Campo FTC: centro 0,0 | ~±1.83m)", "");
            } else {
                telemetry.addData("--- Limelight (diagnostico) ---", "");
                telemetry.addData("Motivo (por que nao usa visao)", kalmanFilter.getLastRejectionReason());
                telemetry.addData("Tags vistas pela LL", kalmanFilter.getLastFiducialIdsSeen());
            }
        }

        telemetry.addData("--- Controles ---", "");
        telemetry.addData("B", "Reset pose");
        telemetry.addData("X", "Toggle fusao");
        telemetry.addData("(Se visao nao entra)", "veja Motivo e Tags vistas acima");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (kalmanFilter != null) {
            kalmanFilter.stop();
        }
    }
}
