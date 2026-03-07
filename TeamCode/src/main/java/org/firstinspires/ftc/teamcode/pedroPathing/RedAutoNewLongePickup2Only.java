package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.teamcode.drive.national.objects.ShootingZones;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Autônomo Red Longe (só Pickup2 x2): score do preload + duas vezes grabPickup2 e scorePickup2.
 * Mesma lógica do Red Longe, mas sem pickup1; turret travada (não se mexe).
 */
@Autonomous(name = "Red Longe 2x Pickup2", group = "National")
public class RedAutoNewLongePickup2Only extends OpMode {

    private Follower follower;
    private RobotHardwareNacional robot;
    private Timer pathTimer, opmodeTimer;

    private int pathState;
    private double scorePhaseStartTime = 0.0;
    private int scoreSubstate = 0;
    private int pickup2Count = 0;

    private static final double STOP_BEFORE_SHOOT_SEC = 1.0;
    private static final double STABILIZE_SEC = 0.4;
    private static final double FIRST_SHOT_STABILIZE_SEC = 2.0;
    private static final double FLAP_CYCLE_SEC = 2.5;
    private static final double AUTO_MAX_DRIVE_POWER = 0.95;
    private static final double AUTO_HOOD_POSITION_LONGE = ConstantsConf.Nacional.HOOD_POSITION_AUTO_LONGE;

    private final Pose startPose = new Pose(88, 7, Math.toRadians(0));
    private final Pose scorePose = new Pose(88, 9, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(134, 9, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(114, 9, Math.toRadians(0));

    private Path scorePreload;
    private PathChain grabPickup2, scorePickup2, end;

    private DcMotorEx leftFlywheel, rightFlywheel;
    private VoltageSensor voltageSensor;
    private DcMotorEx intakeMotor;
    private DcMotorEx intakeMotor2;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        end = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        if (pState == 10) {
            scorePhaseStartTime = getRuntime();
            scoreSubstate = 0;
        }
        if (pState == 12 || pState == 14) scorePhaseStartTime = 0;
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
                    setPathState(12);
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                    follower.followPath(grabPickup2);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                    follower.followPath(scorePickup2, 1, true);
                    setPathState(16);
                }
                break;

            case 16:
                if (!follower.isBusy()) {
                    setPathState(14);
                }
                break;

            case 14:
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
                    pickup2Count++;
                    if (pickup2Count < 2) {
                        follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                        follower.followPath(grabPickup2);
                        setPathState(15);
                    } else {
                        follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                        follower.followPath(end, 1, true);
                        setPathState(7);
                    }
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
            double lockedDeg = ConstantsConf.Nacional.AUTO_TURRET_LOCKED_ANGLE_RED_LONGE_DEG;
            robot.turret.resetAngle(lockedDeg);
            robot.turret.lockAngle(lockedDeg);
        }
        if (robot.hood != null && robot.hood.isEnabled()) {
            robot.hood.setPositionOverride(AUTO_HOOD_POSITION_LONGE);
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
        robot.setTargetPosition(ShootingZones.getRedGoalX(), ShootingZones.getRedGoalY());
        robot.updateWithoutShooter();

        if (intakeMotor != null) intakeMotor.setPower(ConstantsConf.Intake.INTAKE_POWER);
        if (intakeMotor2 != null) intakeMotor2.setPower(ConstantsConf.Intake.INTAKE_POWER);

        double targetTicksPerSec = rpmToTicksPerSecond(ConstantsConf.Nacional.AUTO_SHOOTER_RPM_LONGE);
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
