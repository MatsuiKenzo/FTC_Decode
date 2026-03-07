package org.firstinspires.ftc.teamcode.drive.national.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.national.hardware.RobotHardwareNacional;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Autônomo Red Perto: espelhado do Blue Perto (state machine + PedroPathing).
 * Poses espelhadas em X (144 - x) e heading (π - θ). Mesma rotina: score preload → pickup1 (mid→pickup1) → score → gate → score → pickup2 → score → end.
 */
@Autonomous(name = "Red Perto", group = "National")
public class RedAutoNewPerto extends OpMode {

    private static final double FIELD_WIDTH = 144.0;

    private Follower follower;
    private RobotHardwareNacional robot;
    private Timer pathTimer, opmodeTimer;

    private int pathState;
    /** Estados 10, 3, 12, 6 = em score (esperar estabilizar, atirar, esperar flap). */
    private double scorePhaseStartTime = 0.0;
    private int scoreSubstate = 0;
    private double collectWaitStartTime = 0.0;

    private static final double STOP_BEFORE_SHOOT_SEC = 1.0;
    private static final double STABILIZE_SEC = 0;
    private static final double FIRST_SHOT_STABILIZE_SEC = 2.0;
    private static final double FLAP_CYCLE_SEC = 1.0;
    private static final double COLLECT_WAIT_SEC = 1.0;
    private static final double AUTO_MAX_DRIVE_POWER = 1.0;
    private static final double GRAB_PICKUP2_DRIVE_POWER = 0.8;
    private static final double AUTO_HOOD_POSITION = 1;

    // Poses espelhadas do Blue: x_red = 144 - x_blue, heading_red = π - heading_blue
    private final Pose startPose = new Pose(FIELD_WIDTH - 37, 134, Math.toRadians(90));
    private final Pose scorePose = new Pose(FIELD_WIDTH - 60, 85.234, Math.toRadians(0));
    private final Pose pickup1MidPose = new Pose(FIELD_WIDTH - 41.033, 60, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(FIELD_WIDTH - 12, 60, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(FIELD_WIDTH - 18, 85.234, Math.toRadians(0));
    private final Pose goForGatePose = new Pose(FIELD_WIDTH - 60, 60, Math.toRadians(0));
    private final Pose openGatePose = new Pose(FIELD_WIDTH - 9, 60, Math.toRadians(60)); // π - 120° = 60°
    /** Ponto de controle openGate→score para contornar a bola (espelhado do Blue). */
    private final Pose openGateToScoreControlPoint = new Pose(FIELD_WIDTH - 32, 78, 0);
    private final Pose endPose = new Pose(FIELD_WIDTH - 39, 80, Math.toRadians(0));

    // Rotina: start→score → pickup1Mid→pickup1 → pickup1→score → score→goForGate→openGate → openGate→score → score→pickup2 → pickup2→score → end

    private Path scorePreload;
    private PathChain grabPickup1a, grabPickup1b, scorePickup1;
    private PathChain setToGate, openGate, scorePickup3;
    private PathChain grabPickup2, scorePickup2, end;

    private DcMotorEx leftFlywheel, rightFlywheel;
    private VoltageSensor voltageSensor;
    private DcMotorEx intakeMotor;
    private DcMotorEx intakeMotor2;

    public void buildPaths() {
        // Path1: start → score (preload). Blue (37,134)->(59.664,85.234) 90->180
        double scorePreloadEndX = FIELD_WIDTH - 59.664;
        scorePreload = new Path(new BezierLine(
                new Pose(FIELD_WIDTH - 37, 134),
                new Pose(scorePreloadEndX, 85.234)));
        scorePreload.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0));

        // Path5: score → pickup1Mid. Blue (59.664,85.234)->(41.033,60) 180->180
        grabPickup1a = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(scorePreloadEndX, 85.234), pickup1MidPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1MidPose.getHeading())
                .build();
        // Path6: pickup1Mid → pickup1
        grabPickup1b = follower.pathBuilder()
                .addPath(new BezierLine(pickup1MidPose, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1MidPose.getHeading(), pickup1Pose.getHeading())
                .build();

        // Path4: pickup1 → score
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        // score → goForGate
        setToGate = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, goForGatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goForGatePose.getHeading())
                .build();

        // goForGate → openGate
        openGate = follower.pathBuilder()
                .addPath(new BezierLine(goForGatePose, openGatePose))
                .setLinearHeadingInterpolation(goForGatePose.getHeading(), openGatePose.getHeading())
                .build();

        // openGate → score, com ponto de controle para contornar a bola
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(openGatePose, openGateToScoreControlPoint, scorePose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), scorePose.getHeading())
                .build();

        // score → pickup2
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        // pickup2 → score
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        // score → park (end)
        end = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        if (pState == 10) {
            scorePhaseStartTime = getRuntime();
            scoreSubstate = 0;
        }
        if (pState == 3 || pState == 12 || pState == 6) scorePhaseStartTime = 0;
        if (pState == 2) collectWaitStartTime = getRuntime();
        if (pState == 4 || pState == 5) collectWaitStartTime = 0;
    }

    public void autonomousPathUpdate() {
        double elapsed = scorePhaseStartTime > 0 ? getRuntime() - scorePhaseStartTime : 0;

        switch (pathState) {
            case 0:
                follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                follower.followPath(scorePreload, true);
                setPathState(10);
                break;

            case 10:
                if (scoreSubstate == 0 && elapsed >= STOP_BEFORE_SHOOT_SEC) scoreSubstate = 1;
                if (scoreSubstate == 1 && elapsed >= STOP_BEFORE_SHOOT_SEC + FIRST_SHOT_STABILIZE_SEC) {
                    if (robot != null && robot.intake != null) robot.intake.shoot(true);
                    scoreSubstate = 2;
                }
                if (scoreSubstate == 2 && elapsed >= STOP_BEFORE_SHOOT_SEC + FIRST_SHOT_STABILIZE_SEC + FLAP_CYCLE_SEC) {
                    follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                    follower.followPath(grabPickup1a, 1, true);
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                    follower.followPath(grabPickup1b, 1, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (getRuntime() - collectWaitStartTime >= COLLECT_WAIT_SEC && !follower.isBusy()) {
                    follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                    follower.followPath(scorePickup1, 1, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (follower.isBusy()) break;
                if (scorePhaseStartTime == 0) {
                    scorePhaseStartTime = getRuntime();
                    scoreSubstate = 0;
                }
                elapsed = getRuntime() - scorePhaseStartTime;
                if (scoreSubstate == 0 && elapsed >= STOP_BEFORE_SHOOT_SEC) scoreSubstate = 1;
                if (scoreSubstate == 1 && elapsed >= STOP_BEFORE_SHOOT_SEC + STABILIZE_SEC) {
                    if (robot != null && robot.intake != null) robot.intake.shoot(true);
                    scoreSubstate = 2;
                }
                if (scoreSubstate == 2 && elapsed >= STOP_BEFORE_SHOOT_SEC + STABILIZE_SEC + FLAP_CYCLE_SEC) {
                    follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                    follower.followPath(setToGate);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                    follower.followPath(openGate);
                    setPathState(14);
                }
                break;

            case 14:
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    if (collectWaitStartTime == 0) collectWaitStartTime = getRuntime();
                    if (getRuntime() - collectWaitStartTime >= COLLECT_WAIT_SEC) {
                        follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                        follower.followPath(scorePickup3, 1, true);
                        setPathState(6);
                    }
                }
                break;

            case 6:
                if (follower.isBusy()) break;
                if (scorePhaseStartTime == 0) {
                    scorePhaseStartTime = getRuntime();
                    scoreSubstate = 0;
                }
                elapsed = getRuntime() - scorePhaseStartTime;
                if (scoreSubstate == 0 && elapsed >= STOP_BEFORE_SHOOT_SEC) scoreSubstate = 1;
                if (scoreSubstate == 1 && elapsed >= STOP_BEFORE_SHOOT_SEC + STABILIZE_SEC) {
                    if (robot != null && robot.intake != null) robot.intake.shoot(true);
                    scoreSubstate = 2;
                }
                if (scoreSubstate == 2 && elapsed >= STOP_BEFORE_SHOOT_SEC + STABILIZE_SEC + FLAP_CYCLE_SEC) {
                    follower.setMaxPower(GRAB_PICKUP2_DRIVE_POWER);
                    follower.followPath(grabPickup2, 1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    if (collectWaitStartTime == 0) collectWaitStartTime = getRuntime();
                    if (getRuntime() - collectWaitStartTime >= COLLECT_WAIT_SEC) {
                        follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                        follower.followPath(scorePickup2);
                        setPathState(12);
                    }
                }
                break;

            case 12:
                if (follower.isBusy()) break;
                if (scorePhaseStartTime == 0) {
                    scorePhaseStartTime = getRuntime();
                    scoreSubstate = 0;
                }
                elapsed = getRuntime() - scorePhaseStartTime;
                if (scoreSubstate == 0 && elapsed >= STOP_BEFORE_SHOOT_SEC) scoreSubstate = 1;
                if (scoreSubstate == 1 && elapsed >= STOP_BEFORE_SHOOT_SEC + STABILIZE_SEC) {
                    if (robot != null && robot.intake != null) robot.intake.shoot(true);
                    scoreSubstate = 2;
                }
                if (scoreSubstate == 2 && elapsed >= STOP_BEFORE_SHOOT_SEC + STABILIZE_SEC + FLAP_CYCLE_SEC) {
                    follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                    follower.followPath(end, 1, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
        follower.setMaxPower(AUTO_MAX_DRIVE_POWER);

        robot = new RobotHardwareNacional(hardwareMap, follower);

        try {
            leftFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_LEFT_MOTOR_NAME);
            rightFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_RIGHT_MOTOR_NAME);
            leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
            PIDFCoefficients pidf = new PIDFCoefficients(
                    ConstantsConf.Shooter.KP, ConstantsConf.Shooter.KI,
                    ConstantsConf.Shooter.KD, ConstantsConf.Shooter.KF);
            leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        } catch (Exception e) {
            leftFlywheel = null;
            rightFlywheel = null;
        }
        try {
            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception e) {
            voltageSensor = null;
        }

        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, ConstantsConf.Intake.INTAKE_MOTOR_NAME);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            intakeMotor = null;
        }
        try {
            intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake_2");
            intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            intakeMotor2 = null;
        }

        if (robot.turret != null) {
            double lockedDeg = ConstantsConf.Nacional.AUTO_TURRET_LOCKED_ANGLE_RED_DEG;
            robot.turret.resetAngle(lockedDeg);
            robot.turret.lockAngle(lockedDeg);
        }

        if (robot.hood != null && robot.hood.isEnabled()) {
            robot.hood.setPositionOverride(AUTO_HOOD_POSITION);
        }
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        robot.updateWithoutShooter();

        if (robot.turret != null) robot.turret.lockAngle(ConstantsConf.Nacional.AUTO_TURRET_LOCKED_ANGLE_RED_DEG);

        if (intakeMotor != null) intakeMotor.setPower(ConstantsConf.Intake.INTAKE_POWER);
        if (intakeMotor2 != null) intakeMotor2.setPower(ConstantsConf.Intake.INTAKE_POWER);

        double targetTicksPerSec = rpmToTicksPerSecond(ConstantsConf.Nacional.AUTO_SHOOTER_RPM);
        double voltageCompensation = 1.0;
        if (voltageSensor != null && ConstantsConf.Shooter.NOMINAL_VOLTAGE > 0 && voltageSensor.getVoltage() > 0.5) {
            voltageCompensation = ConstantsConf.Shooter.NOMINAL_VOLTAGE / voltageSensor.getVoltage();
        }
        double velocityToSet = targetTicksPerSec * voltageCompensation;
        if (leftFlywheel != null && rightFlywheel != null) {
            leftFlywheel.setVelocity(velocityToSet);
            rightFlywheel.setVelocity(velocityToSet);
        }

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (leftFlywheel != null) leftFlywheel.setVelocity(0);
        if (rightFlywheel != null) rightFlywheel.setVelocity(0);
        if (intakeMotor != null) intakeMotor.setPower(0);
        if (intakeMotor2 != null) intakeMotor2.setPower(0);
        if (robot != null) robot.stop();
    }

    private static double rpmToTicksPerSecond(double rpm) {
        if (ConstantsConf.Shooter.TICKS_PER_REVOLUTION <= 0) return 0;
        return rpm * ConstantsConf.Shooter.TICKS_PER_REVOLUTION / 60.0;
    }
}
