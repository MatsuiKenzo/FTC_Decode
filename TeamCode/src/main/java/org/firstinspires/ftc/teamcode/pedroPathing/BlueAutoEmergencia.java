package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Autônomo de emergência Blue: posicione o robô na mesma start pose do Blue Longe.
 * Apenas vai em linha reta até a pose inicial do TeleOp Blue (13.25, 8.85, 180°).
 * Sem shooter, intake ou turret — só drive. Use só se necessário.
 */
@Autonomous(name = "Blue Emergencia", group = "National")
public class BlueAutoEmergencia extends OpMode {

    private Follower follower;

    /** Mesma start pose do Blue Longe. */
    private final Pose startPose = new Pose(64, 8, Math.toRadians(180));
    /** Pose inicial do TeleOp Blue (para onde o robô vai). */
    private final Pose endPose = new Pose(13.25, 17.7 / 2, Math.toRadians(180));

    private Path pathToTeleOpStart;
    private int pathState = -1; // -1 = fim, 0 = iniciar path, 1 = seguindo

    private static final double AUTO_MAX_DRIVE_POWER = 0.95;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(AUTO_MAX_DRIVE_POWER);

        pathToTeleOpStart = new Path(new BezierLine(startPose, endPose));
        pathToTeleOpStart.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());

        telemetry.addData("Status", "Blue Emergencia: start Longe -> pose TeleOp. So drive.");
        telemetry.update();
    }

    @Override
    public void start() {
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();

        if (pathState == 0) {
            follower.followPath(pathToTeleOpStart, true);
            pathState = 1;
        } else if (pathState == 1 && !follower.isBusy()) {
            pathState = -1;
        }

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading (°)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}
