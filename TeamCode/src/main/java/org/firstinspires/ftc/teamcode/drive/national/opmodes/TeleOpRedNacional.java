package org.firstinspires.ftc.teamcode.drive.national.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.drive.national.actuators.KalmanFilterLocalizer;
import org.firstinspires.ftc.teamcode.drive.national.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.drive.national.hardware.RobotHardwareNacional;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;
import org.firstinspires.ftc.teamcode.drive.util.ShooterDistanceToRPM;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * TeleOp Red Nacional
 *
 * Mesma arquitetura e controles que TeleOp Blue Nacional; pose e goal para aliança vermelha.
 * Shooter/intake como FlapIntakeTester. GP1: drive, LT intake, RT flap, A shooter, B modo, D-Pad vel. GP2: X escala, A turret, Y goal, B reset, RB Limelight.
 */
@TeleOp(name = "TeleOp Red Nacional", group = "Nacional")
public class TeleOpRedNacional extends OpMode {
    private FieldOrientedDrive fod;
    private RobotHardwareNacional robot;
    private Follower follower;
    private KalmanFilterLocalizer kalmanFilter;
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private VoltageSensor voltageSensor;
    private final ShooterDistanceToRPM distanceToRPM = new ShooterDistanceToRPM();

    private boolean useDistanceBasedVelocity = false;
    private double curTargetVelocity = 0;
    private boolean shooterActive = false;
    private double scaleFactor = 10.0;
    private int scaleMode = 0;

    private boolean turretLocked = false;
    private boolean a1Prev = false, b1Prev = false, a2Prev = false, b2Prev = false, y2Prev = false, x2Prev = false, rb2Prev = false;
    private boolean leftTriggerPrev = false;
    private boolean rightTriggerPrev = false;
    private boolean leftBumperPrev = false;
    private boolean shooterWasReady = false;

    private final Pose startTeleop = new Pose(105, 80, Math.toRadians(180));
    private double targetX = 138.0;
    private double targetY = 138.0;

    @Override
    public void init() {
        fod = new FieldOrientedDrive(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startTeleop);

        if (ConstantsConf.KalmanLocalizer.ENABLED) {
            kalmanFilter = new KalmanFilterLocalizer();
            if (kalmanFilter.init(hardwareMap, follower)) {
                telemetry.addData("Kalman", "Limelight + Pinpoint ativo");
            } else {
                kalmanFilter = null;
            }
        } else {
            kalmanFilter = null;
        }

        distanceToRPM.buildFromConstants();

        robot = new RobotHardwareNacional(hardwareMap, follower);
        robot.setTargetPosition(targetX, targetY);

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

        if (robot.hood != null && robot.hood.isEnabled()) {
            telemetry.addData("Hood/Tilt", "ATIVO");
        } else {
            telemetry.addData("Hood/Tilt", "DESATIVADO");
        }
        telemetry.addData("Status", "Inicializado. Red Nacional. GP1: drive, LT intake, RT flap, A shooter, B modo, D-Pad. GP2: X escala, A turret, Y goal, B reset, RB Limelight.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        if (kalmanFilter != null) {
            kalmanFilter.update();
        }
        robot.updateWithoutShooter();

        boolean lbNow = gamepad1.left_bumper;
        boolean resetIMUThisLoop = lbNow && !leftBumperPrev;
        leftBumperPrev = lbNow;
        fod.movement(
            -gamepad1.left_stick_x,
            gamepad1.left_stick_y,
            gamepad1.right_stick_x,
            resetIMUThisLoop
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

        boolean a1Now = gamepad1.a;
        if (a1Now && !a1Prev) {
            shooterActive = !shooterActive;
        }
        a1Prev = a1Now;

        boolean b1Now = gamepad1.b;
        if (b1Now && !b1Prev) {
            useDistanceBasedVelocity = !useDistanceBasedVelocity;
        }
        b1Prev = b1Now;

        double effectiveVelocity = curTargetVelocity;
        double distanceToGoal = 0.0;
        if (robot.shooter != null) {
            distanceToGoal = robot.shooter.getDistance();
            if (useDistanceBasedVelocity && Double.isFinite(distanceToGoal) && distanceToGoal >= 1.0) {
                double rpm = distanceToRPM.getRPM(distanceToGoal);
                effectiveVelocity = rpmToTicksPerSecond(rpm);
            }
        }

        double voltageCompensation = 1.0;
        if (voltageSensor != null && ConstantsConf.Shooter.NOMINAL_VOLTAGE > 0) {
            double currentV = voltageSensor.getVoltage();
            if (currentV > 0.5) {
                voltageCompensation = ConstantsConf.Shooter.NOMINAL_VOLTAGE / currentV;
            }
        }
        double velocityToSet = effectiveVelocity * voltageCompensation;

        if (leftFlywheel != null && rightFlywheel != null) {
            if (gamepad1.dpad_up) {
                curTargetVelocity += scaleFactor;
                curTargetVelocity = Math.min(6000, curTargetVelocity);
            }
            if (gamepad1.dpad_down) {
                curTargetVelocity -= scaleFactor;
                curTargetVelocity = Math.max(0, curTargetVelocity);
            }

            if (shooterActive) {
                leftFlywheel.setVelocity(velocityToSet);
                rightFlywheel.setVelocity(velocityToSet);
            } else {
                leftFlywheel.setVelocity(0);
                rightFlywheel.setVelocity(0);
            }
        }

        boolean x2Now = gamepad2.x;
        if (x2Now && !x2Prev) {
            scaleMode = (scaleMode + 1) % 3;
            switch (scaleMode) {
                case 0: scaleFactor = 10.0; break;
                case 1: scaleFactor = 100.0; break;
                case 2: scaleFactor = 1000.0; break;
            }
        }
        x2Prev = x2Now;

        boolean b2Now = gamepad2.b;
        if (b2Now && !b2Prev) {
            follower.setPose(startTeleop);
        }
        b2Prev = b2Now;

        boolean y2Now = gamepad2.y;
        if (y2Now && !y2Prev && robot.turret != null) {
            Pose robotPose = follower.getPose();
            double dist = robot.shooter.getDistance();
            double headingRad = robotPose.getHeading();
            double turretDeg = robot.turret.getMotorAngle();
            double absoluteAngleRad = headingRad + Math.toRadians(turretDeg);
            double newTargetX = robotPose.getX() + dist * Math.cos(absoluteAngleRad);
            double newTargetY = robotPose.getY() + dist * Math.sin(absoluteAngleRad);
            robot.setTargetPosition(newTargetX, newTargetY);
        }
        y2Prev = y2Now;

        boolean a2Now = gamepad2.a;
        if (a2Now && !a2Prev && robot.turret != null) {
            turretLocked = !turretLocked;
        }
        a2Prev = a2Now;
        if (robot.turret != null) {
            if (turretLocked) {
                robot.turret.lockAngle(-45.0);
            } else {
                robot.turret.unlockAngle();
            }
        }

        boolean rb2Now = gamepad2.right_bumper;
        if (rb2Now && !rb2Prev && kalmanFilter != null) {
            kalmanFilter.setVisionFusionEnabled(!kalmanFilter.isVisionFusionEnabled());
        }
        rb2Prev = rb2Now;

        if (leftFlywheel != null && rightFlywheel != null && shooterActive && useDistanceBasedVelocity) {
            double avgVel = (leftFlywheel.getVelocity() + rightFlywheel.getVelocity()) / 2.0;
            boolean ready = Math.abs(avgVel - velocityToSet) < 80;
            if (ready && !shooterWasReady) {
                gamepad1.rumble(1.0, 1.0, 250);
            }
            shooterWasReady = ready;
        } else {
            shooterWasReady = false;
        }

        telemetry.addData("--- Shooter ---", "");
        telemetry.addData("Modo (B GP1)", useDistanceBasedVelocity ? "KALMAN" : "MANUAL");
        telemetry.addData("Active (A GP1)", shooterActive ? "ON" : "OFF");
        telemetry.addData("Distance (pol)", "%.1f", distanceToGoal);
        telemetry.addData("Target vel", "%.0f", effectiveVelocity);
        telemetry.addData("Vel comp. tensão", "%.0f", velocityToSet);
        if (voltageSensor != null) {
            telemetry.addData("Battery", "%.2f V", voltageSensor.getVoltage());
        }
        telemetry.addData("Scale (X GP2)", "%.0f", scaleFactor);
        if (leftFlywheel != null) {
            telemetry.addData("Current L/R", "%.0f / %.0f", leftFlywheel.getVelocity(), rightFlywheel != null ? rightFlywheel.getVelocity() : 0);
        }
        if (robot.turret != null) {
            telemetry.addData("Turret", "%.1f° Travada=%s", robot.turret.getMotorAngle(), turretLocked ? "SIM" : "NÃO");
        }
        if (robot.hood != null && robot.hood.isEnabled()) {
            telemetry.addData("Hood", "%.2f", robot.hood.getCurrentAngle());
        }
        telemetry.addData("Pose", "%.1f, %.1f", follower.getPose().getX(), follower.getPose().getY());
        if (kalmanFilter != null) {
            telemetry.addData("Fusao (RB GP2)", kalmanFilter.isVisionFusionEnabled() ? "ON" : "OFF");
        }
        telemetry.addData("Intake", robot.intake.isIntakeActive() ? "ON" : "OFF");
        telemetry.update();
    }

    private double rpmToTicksPerSecond(double rpm) {
        if (ConstantsConf.Shooter.TICKS_PER_REVOLUTION <= 0) return 0;
        return rpm * ConstantsConf.Shooter.TICKS_PER_REVOLUTION / 60.0;
    }

    @Override
    public void stop() {
        if (kalmanFilter != null) {
            kalmanFilter.stop();
        }
        if (leftFlywheel != null) leftFlywheel.setVelocity(0);
        if (rightFlywheel != null) rightFlywheel.setVelocity(0);
        robot.stop();
    }
}
