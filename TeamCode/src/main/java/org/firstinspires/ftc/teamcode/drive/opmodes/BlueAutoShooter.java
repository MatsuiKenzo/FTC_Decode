package org.firstinspires.ftc.teamcode.drive.opmodes;

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

import org.firstinspires.ftc.teamcode.drive.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Autônomo Azul: paths do JSON, tiro de 3 bolas na pose (60, 85) e intake em Path 2 e Path 5.
 * Torreta mira no Blue Goal o tempo inteiro
 *
 * - Start: (20.34, 123.29), heading 90°
 * - Path 1 → (60, 85): shooting pose → estabiliza e atira 3 bolas no Blue Goal
 * - Path 2 (60,85)→(37,85): velocidade reduzida + intake para coletar 3 bolas
 * - Path 3 → (12, 84.44)
 * - Path 4 → (60, 85): shooting pose → atira 3 bolas
 * - Path 5 (60,85)→(37,60): velocidade reduzida + intake
 * - Path 6 → (12, 59.79)
 * - Path 7 → (60, 85): shooting pose → atira 3 bolas
 */
@Autonomous(name = "Blue Auto Shooter", group = "Auto")
public class BlueAutoShooter extends com.seattlesolvers.solverslib.command.CommandOpMode {
    private Follower follower;
    private RobotHardware robot;
    private TelemetryData telemetryData;

    // Blue Goal (alvo do tiro) - centro aproximado do triângulo azul
    private static final double BLUE_GOAL_X = 7.0;
    private static final double BLUE_GOAL_Y = 107.0;

    // Poses do JSON (headings em radianos)
    private final Pose startPose = new Pose(20.336, 123.29, Math.toRadians(90));
    private final Pose shootingPose = new Pose(60.0, 85.0, Math.toRadians(180));
    private final Pose pose2 = new Pose(37, 85, Math.toRadians(180));
    private final Pose pose3 = new Pose(12, 84.439, Math.toRadians(180));
    private final Pose pose5 = new Pose(37, 60, Math.toRadians(180));
    private final Pose pose6 = new Pose(12, 59.794, Math.toRadians(180));

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

    /** Espera estabilizar e atira 3 bolas (indexer). Alvo já definido no init. */
    private SequentialCommandGroup shoot3Balls() {
        return new SequentialCommandGroup(
                new WaitCommand(800),
                new InstantCommand(() -> robot.intake.setIndexerPower(1.0)),
                new WaitCommand(600),
                new InstantCommand(() -> robot.intake.setIndexerPower(0.0)),
                new WaitCommand(400),
                new InstantCommand(() -> robot.intake.setIndexerPower(1.0)),
                new WaitCommand(600),
                new InstantCommand(() -> robot.intake.setIndexerPower(0.0)),
                new WaitCommand(400),
                new InstantCommand(() -> robot.intake.setIndexerPower(1.0)),
                new WaitCommand(600),
                new InstantCommand(() -> {
                    robot.intake.setIndexerPower(0.0);
                    robot.shooter.stop();
                })
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
        robot = new RobotHardware(hardwareMap, follower);
        robot.shooter.setUseDistanceBasedVelocity(true);

        robot.setTargetPosition(BLUE_GOAL_X, BLUE_GOAL_Y);
        buildPaths();

        schedule(
                new FollowPathCommand(follower, path1),
                shoot3Balls(),

                new FollowPathCommand(follower, path2),

                startIntake(),
                new FollowPathCommand(follower, path3, true, 0.5),
                stopIntake(),

                new FollowPathCommand(follower, path4),
                shoot3Balls(),

                new FollowPathCommand(follower, path5),

                startIntake(),
                new FollowPathCommand(follower, path6, true, 0.5),
                stopIntake(),

                new FollowPathCommand(follower, path7),
                shoot3Balls()
        );
    }

    @Override
    public void run() {
        super.run();
        follower.update();
        robot.update();

        telemetryData.addData("X", follower.getPose().getX());
        telemetryData.addData("Y", follower.getPose().getY());
        telemetryData.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryData.update();
    }
}