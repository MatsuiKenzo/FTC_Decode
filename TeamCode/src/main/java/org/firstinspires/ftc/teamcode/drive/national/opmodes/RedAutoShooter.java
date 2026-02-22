package org.firstinspires.ftc.teamcode.drive.national.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.TelemetryData;

import org.firstinspires.ftc.teamcode.drive.national.actuators.KalmanFilterLocalizer;
import org.firstinspires.ftc.teamcode.drive.national.hardware.RobotHardwareNacional;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Versão espelhada do BlueAutoShooter para a aliança vermelha (robô NACIONAL).
 *
 * Mesma lógica: flap para atirar, intake ligado durante coleta, turret/shooter atualizando no run().
 * Poses/goal espelhadas em X (campo 144\").
 *
 * - Start: (~123.66, 123.29), heading 90°
 * - Path 1 → (~84, 85): shooting pose → atira 3 bolas no Red Goal
 * - Path 2+3: intake ligado, coleta
 * - Path 4, 5+6, 7: mesmo padrão do BlueAutoShooter.
 */
@Autonomous(name = "Red Auto Shooter", group = "Auto")
public class RedAutoShooter extends com.seattlesolvers.solverslib.command.CommandOpMode {
    private Follower follower;
    private RobotHardwareNacional robot;
    private KalmanFilterLocalizer kalmanFilter;
    private TelemetryData telemetryData;

    // Red Goal (espelho do BLUE_GOAL_X/Y do BlueAutoShooter)
    private static final double RED_GOAL_X = 137.0;  // 144 - 7
    private static final double RED_GOAL_Y = 107.0;

    // Poses espelhadas em X em relação a 144
    private final Pose startPose = new Pose(123.664, 123.29, Math.toRadians(90));      // 144 - 20.336
    private final Pose shootingPose = new Pose(84.0, 85.0, Math.toRadians(0));        // 144 - 60, heading espelhado
    private final Pose pose2 = new Pose(107.0, 85.0, Math.toRadians(0));              // 144 - 37
    private final Pose pose3 = new Pose(132.0, 84.439, Math.toRadians(0));            // 144 - 12
    private final Pose pose5 = new Pose(107.0, 60.0, Math.toRadians(0));              // 144 - 37
    private final Pose pose6 = new Pose(132.0, 59.794, Math.toRadians(0));            // 144 - 12

    private PathChain path1, path2, path3, path4, path5, path6, path7;

    public void buildPaths() {
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootingPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootingPose.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, pose2))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), pose2.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(pose2, pose3))
                .setLinearHeadingInterpolation(pose2.getHeading(), pose3.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(pose3, shootingPose))
                .setLinearHeadingInterpolation(pose3.getHeading(), shootingPose.getHeading())
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(shootingPose, pose5))
                .setLinearHeadingInterpolation(shootingPose.getHeading(), pose5.getHeading())
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(pose5, pose6))
                .setLinearHeadingInterpolation(pose5.getHeading(), pose6.getHeading())
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(pose6, shootingPose))
                .setLinearHeadingInterpolation(pose6.getHeading(), shootingPose.getHeading())
                .build();
    }

    /** Ciclo do flap: 0.25s + 2s (3 bolas saem) + 0.25s = 2.5s. */
    private static final int FLAP_CYCLE_MS = 2500;

    /** Estabiliza, um ciclo do flap (2s alinhado = 3 bolas), depois para o shooter. */
    private SequentialCommandGroup shoot3Balls() {
        return new SequentialCommandGroup(
                new WaitCommand(800),
                new InstantCommand(() -> robot.intake.shoot(true)),
                new WaitCommand(FLAP_CYCLE_MS),
                new InstantCommand(() -> robot.shooter.stop())
        );
    }

    private InstantCommand startIntake() {
        return new InstantCommand(() -> robot.intake.setPower(ConstantsConf.Intake.INTAKE_POWER));
    }

    private InstantCommand stopIntake() {
        return new InstantCommand(() -> robot.intake.stop());
    }

    @Override
    public void initialize() {
        super.reset();
        telemetryData = new TelemetryData(telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        if (ConstantsConf.KalmanLocalizer.ENABLED) {
            kalmanFilter = new KalmanFilterLocalizer();
            kalmanFilter.init(hardwareMap, follower);
        } else {
            kalmanFilter = null;
        }

        robot = new RobotHardwareNacional(hardwareMap, follower);
        robot.shooter.setUseDistanceBasedVelocity(true);

        robot.setTargetPosition(RED_GOAL_X, RED_GOAL_Y);
        buildPaths();

        schedule(
                new FollowPathCommand(follower, path1),
                shoot3Balls(),

                startIntake(),
                new FollowPathCommand(follower, path2),
                new FollowPathCommand(follower, path3, true, 0.5),
                stopIntake(),

                new FollowPathCommand(follower, path4),
                shoot3Balls(),

                startIntake(),
                new FollowPathCommand(follower, path5),
                new FollowPathCommand(follower, path6, true, 0.5),
                stopIntake(),

                new FollowPathCommand(follower, path7),
                shoot3Balls(),

                // Cleanup no final da sequência
                new InstantCommand(() -> {
                    if (kalmanFilter != null) kalmanFilter.stop();
                    if (robot != null) robot.stop();
                })
        );
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        if (kalmanFilter != null) {
            kalmanFilter.update();
        }
        robot.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryData.update();
    }
}

