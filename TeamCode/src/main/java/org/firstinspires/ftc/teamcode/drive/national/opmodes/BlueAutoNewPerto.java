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
 * Autônomo Blue Perto: movimentação igual à classe original (state machine + PedroPathing).
 * Intake por controle direto dos motores; shooter setVelocity como no TeleOp; turret travada; hood fixo em 1.0.
 */
@Autonomous(name = "Blue Perto", group = "National")
public class BlueAutoNewPerto extends OpMode {

    private Follower follower;
    private RobotHardwareNacional robot;
    private Timer pathTimer, opmodeTimer;

    private int pathState;
    /** Estados 10, 3, 12, 6 = em score (esperar estabilizar, atirar, esperar flap). */
    private double scorePhaseStartTime = 0.0;
    private int scoreSubstate = 0; // 0 = parado 1s, 1 = estabilizar depois atirar, 2 = ciclo do flap
    private double collectWaitStartTime = 0.0;

    private static final double STOP_BEFORE_SHOOT_SEC = 1.0;
    private static final double STABILIZE_SEC = 0.4;
    private static final double FIRST_SHOT_STABILIZE_SEC = 2.0;
    private static final double FLAP_CYCLE_SEC = 2.5;
    private static final double COLLECT_WAIT_SEC = 1.0;
    /** Potência máxima em todos os paths (reta e diagonal). Aumentar se diagonal ficar devagar. */
    private static final double AUTO_MAX_DRIVE_POWER = 0.95;
    /** Hood no auto: um pouco mais alto que 1.0 (ajuste 0.85–0.95 conforme o robô). */
    private static final double AUTO_HOOD_POSITION = 1;

    // Poses (iguais ao original)
    private final Pose startPose = new Pose(37, 134, Math.toRadians(90));
    private final Pose scorePose = new Pose(60, 85, Math.toRadians(180));

    /** Mais para frente = menos X. Ajuste se precisar. */
    private final Pose pickup1Pose = new Pose(19, 85, Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(18, 60, Math.toRadians(180));
    private final Pose goForGatePose = new Pose(60, 60, Math.toRadians(180));
    private final Pose openGatePose = new Pose(9, 58, Math.toRadians(135));
    private final Pose pickup3Pose = new Pose(39, 80, Math.toRadians(180));
    private final Pose controlPoint = new Pose(65, 55, Math.toRadians(180));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2;
    private PathChain setToGate, openGate, scorePickup3, end;

    private DcMotorEx leftFlywheel, rightFlywheel;
    private VoltageSensor voltageSensor;
    private DcMotorEx intakeMotor;
    private DcMotorEx intakeMotor2;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, controlPoint, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        setToGate = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, goForGatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), goForGatePose.getHeading())
                .build();

        openGate = follower.pathBuilder()
                .addPath(new BezierLine(goForGatePose, openGatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), openGatePose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, scorePose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), scorePose.getHeading())
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
        if (pState == 3 || pState == 12 || pState == 6) scorePhaseStartTime = 0; // só inicia ao chegar (!isBusy)
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
                    follower.followPath(grabPickup1, 1, true);
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
                    follower.followPath(grabPickup2, 1, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    if (collectWaitStartTime == 0) collectWaitStartTime = getRuntime();
                    if (getRuntime() - collectWaitStartTime >= COLLECT_WAIT_SEC) {
                        follower.setMaxPower(AUTO_MAX_DRIVE_POWER); // Aqui
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
                    follower.followPath(setToGate);
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.setMaxPower(AUTO_MAX_DRIVE_POWER);
                    follower.followPath(openGate);
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

        // Shooter: igual ao TeleOp (setVelocity + PIDF)
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

        // Intake: controle direto dos motores (como no TeleOp)
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, ConstantsConf.Intake.INTAKE_MOTOR_NAME);
            intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            intakeMotor = null;
        }
        try {
            intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake_2");
            intakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            intakeMotor2 = null;
        }

        // Turret: 0° = costas, travada no ângulo do auto
        if (robot.turret != null) {
            robot.turret.resetAngle(0.0);
            robot.turret.lockAngle(ConstantsConf.Nacional.AUTO_TURRET_LOCKED_ANGLE_BLUE_DEG);
        }

        // Hood: fixo na pose normal (1.0) o autônomo todo
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

        // Turret: reaplicar ângulo travado todo loop (corrige inércia/deriva)
        if (robot.turret != null) robot.turret.lockAngle(ConstantsConf.Nacional.AUTO_TURRET_LOCKED_ANGLE_BLUE_DEG);

        // Intake: potência direta nos motores (todo loop)
        if (intakeMotor != null) intakeMotor.setPower(ConstantsConf.Intake.INTAKE_POWER);
        if (intakeMotor2 != null) intakeMotor2.setPower(ConstantsConf.Intake.INTAKE_POWER);

        // Shooter: setVelocity como no TeleOp, RPM fixo + compensação de tensão
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
