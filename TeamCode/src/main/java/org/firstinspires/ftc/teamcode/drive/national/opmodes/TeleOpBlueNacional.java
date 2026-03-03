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
import org.firstinspires.ftc.teamcode.drive.national.objects.ShootingZones;
import org.firstinspires.ftc.teamcode.drive.national.hardware.RobotHardwareNacional;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;
import org.firstinspires.ftc.teamcode.drive.util.ShooterDistanceToRPM;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * TeleOp Blue Nacional
 *
 * Arquitetura: RobotHardwareNacional. Shooter e intake como no FlapIntakeTester:
 * - Shooter: controle direto (setVelocity), ligar/desligar (A GP1), modo Manual ou Kalman (B GP1), D-Pad ajusta velocidade (manual), escala no GP2 X.
 * - Intake: dois motores (intake + intake_2) mesma potência; toggle no LT (GP1).
 * - Turret/Hood: lógica atual; travar (A GP2), recalibrar goal (Y GP2). Hood só se TILT_ENABLED.
 *
 * Controles:
 * - Gamepad 1: Drive (sticks), Reset IMU (LB), Intake toggle (LT), Flap/atirar (RT), Shooter on/off (A), Modo Manual/Kalman (B), D-Pad velocidade (manual).
 * - Gamepad 2: Escala D-Pad (X), Travar turret (A), Recalibrar goal (Y), Reset pose (B), Fusão Limelight (RB).
 */
@TeleOp(name = "TeleOp Blue Nacional", group = "Nacional")
public class TeleOpBlueNacional extends OpMode {
    private FieldOrientedDrive fod;
    private RobotHardwareNacional robot;
    private Follower follower;
    private KalmanFilterLocalizer kalmanFilter;
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private VoltageSensor voltageSensor;
    private final ShooterDistanceToRPM distanceToRPM = new ShooterDistanceToRPM();

    // Shooter: igual FlapIntakeTester
    private boolean useDistanceBasedVelocity = false; // B GP1: false = Manual, true = Kalman
    private double curTargetVelocity = 0;
    private boolean shooterActive = false;
    private double scaleFactor = 10.0;
    private int scaleMode = 0; // 0=10, 1=100, 2=1000

    private boolean turretLocked = false;
    private boolean a1Prev = false, b1Prev = false, a2Prev = false, b2Prev = false, y2Prev = false, x2Prev = false, rb2Prev = false;
    private boolean rightTriggerPrev = false;
    private boolean leftBumperPrev = false;
    private boolean shooterWasReady = false;

    private final Pose startTeleop = new Pose(39, 80, Math.toRadians(180));
    private double targetX = 6.0;
    private double targetY = 140.0;

    @Override
    public void init() {
        fod = new FieldOrientedDrive(hardwareMap);
        fod.resetYaw();
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
        if (robot.hood != null && robot.hood.isEnabled()) {
            robot.hood.setPositionOverride(1.0); // Hood por zona; init em 1.0
        }
        if (robot.turret != null) {
            // Turret começa na posição em que o autônomo termina (transição auto → TeleOp)
            double turretStartDeg = ConstantsConf.Nacional.AUTO_TURRET_LOCKED_ANGLE_BLUE_DEG;
            robot.turret.resetAngle(turretStartDeg);
            robot.turret.lockAngle(turretStartDeg);
        }

        // Flywheel
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
        telemetry.addData("Status", "Inicializado. GP1: drive, LT intake, RT flap, A shooter, B modo, D-Pad vel. GP2: X escala, A turret, Y goal, B reset, RB Limelight.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        if (kalmanFilter != null) {
            kalmanFilter.update();
        }

        // Zonas: turret mira no goal; hood por zona (red=0.7, blue=1.0); fora das zonas turret em 0° (costas)
        double px = follower.getPose().getX();
        double py = follower.getPose().getY();
        if (robot.turret != null) {
            if (ShootingZones.isInAnyShootingZone(px, py)) {
                robot.setTargetPosition(ShootingZones.getBlueGoalX(), ShootingZones.getBlueGoalY());
                if (turretLocked) robot.turret.lockAngle(-45.0);
                else robot.turret.unlockAngle();
            } else {
                robot.turret.lockAngle(0.0); // Fora das zonas: 0° = costas
            }
        }
        if (robot.hood != null && robot.hood.isEnabled()) {
            if (ShootingZones.isInRedGoalZone(px, py)) robot.hood.setPositionOverride(0.7);
            else if (ShootingZones.isInBlueGoalZone(px, py)) robot.hood.setPositionOverride(1.0);
            else robot.hood.setPositionOverride(1.0);
        }

        robot.updateWithoutShooter();

        // Drive — reset IMU só na borda de subida do LB (evita múltiplos resets segundos)
        boolean lbNow = gamepad1.left_bumper;
        boolean resetIMUThisLoop = lbNow && !leftBumperPrev;
        leftBumperPrev = lbNow;
        fod.movement(
            -gamepad1.left_stick_x,
            gamepad1.left_stick_y,
            gamepad1.right_stick_x,
            resetIMUThisLoop
        );

        // Intake (GP1 LT): só ligado enquanto o trigger estiver segurado (economia de bateria)
        robot.intake.setIntakeFromTrigger(gamepad1.left_trigger > 0.1);

        // Flap (GP1 RT): ao atirar, intake também liga pelo mesmo tempo do ciclo do flap
        boolean rightTriggerNow = gamepad1.right_trigger > 0.1;
        if (rightTriggerNow && !rightTriggerPrev) {
            robot.intake.shoot(true);
        } else {
            robot.intake.shoot(false);
        }
        rightTriggerPrev = rightTriggerNow;

        // GP1 A: shooter on/off
        boolean a1Now = gamepad1.a;
        if (a1Now && !a1Prev) {
            shooterActive = !shooterActive;
        }
        a1Prev = a1Now;

        // GP1 B: modo Manual <-> Kalman
        boolean b1Now = gamepad1.b;
        if (b1Now && !b1Prev) {
            useDistanceBasedVelocity = !useDistanceBasedVelocity;
        }
        b1Prev = b1Now;

        // Velocidade efetiva: Kalman = por distância; Manual = D-Pad
        double effectiveVelocity = curTargetVelocity;
        double distanceToGoal = 0.0;
        if (robot.shooter != null) {
            distanceToGoal = robot.shooter.getDistance();
            if (useDistanceBasedVelocity) {
                if (Double.isFinite(distanceToGoal) && distanceToGoal >= 1.0) {
                    double rpm = distanceToRPM.getRPM(distanceToGoal);
                    effectiveVelocity = rpmToTicksPerSecond(rpm);
                }
            }
        }

        // Compensação de tensão: mesma “efetividade” com bateria baixa
        double voltageCompensation = 1.0;
        if (voltageSensor != null && ConstantsConf.Shooter.NOMINAL_VOLTAGE > 0) {
            double currentV = voltageSensor.getVoltage();
            if (currentV > 0.5) {
                voltageCompensation = ConstantsConf.Shooter.NOMINAL_VOLTAGE / currentV;
            }
        }
        double velocityToSet = effectiveVelocity * voltageCompensation;

        // D-Pad (GP1) ajusta curTargetVelocity (manual)
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

        // GP2 X: escala do D-Pad (10/100/1000)
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

        // GP2 B: reset pose
        boolean b2Now = gamepad2.b;
        if (b2Now && !b2Prev) {
            follower.setPose(startTeleop);
        }
        b2Prev = b2Now;

        // GP2 Y: recalibrar goal (onde está mirando)
        boolean y2Now = gamepad2.y;
        if (y2Now && !y2Prev && robot.turret != null) {
            Pose robotPose = follower.getPose();
            double dist = robot.shooter.getDistance();
            double headingRad = robotPose.getHeading() + Math.toRadians(ConstantsConf.Nacional.DRIVE_HEADING_OFFSET_DEG);
            double turretDeg = robot.turret.getMotorAngle();
            double absoluteAngleRad = headingRad + Math.toRadians(turretDeg);
            double newTargetX = robotPose.getX() + dist * Math.cos(absoluteAngleRad);
            double newTargetY = robotPose.getY() + dist * Math.sin(absoluteAngleRad);
            robot.setTargetPosition(newTargetX, newTargetY);
        }
        y2Prev = y2Now;

        // GP2 A: travar/destravar turret
        boolean a2Now = gamepad2.a;
        if (a2Now && !a2Prev && robot.turret != null) {
            turretLocked = !turretLocked;
        }
        a2Prev = a2Now;

        // GP2 RB: toggle fusão Limelight
        boolean rb2Now = gamepad2.right_bumper;
        if (rb2Now && !rb2Prev && kalmanFilter != null) {
            kalmanFilter.setVisionFusionEnabled(!kalmanFilter.isVisionFusionEnabled());
        }
        rb2Prev = rb2Now;

        // Rumble quando shooter "ready"
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

        // Telemetry
        telemetry.addData("--- Shooter (FlapIntakeTester style) ---", "");
        telemetry.addData("Modo (B GP1)", useDistanceBasedVelocity ? "KALMAN" : "MANUAL");
        telemetry.addData("Active (A GP1)", shooterActive ? "ON" : "OFF");
        telemetry.addData("Distance (pol)", "%.1f", distanceToGoal);
        telemetry.addData("Target vel (ticks/s)", "%.0f", effectiveVelocity);
        telemetry.addData("Vel com comp. tensão", "%.0f", velocityToSet);
        if (voltageSensor != null) {
            telemetry.addData("Battery", "%.2f V (nom %.1f)", voltageSensor.getVoltage(), ConstantsConf.Shooter.NOMINAL_VOLTAGE);
        }
        telemetry.addData("Manual guardado", "%.0f", curTargetVelocity);
        telemetry.addData("Scale (X GP2)", "%.0f", scaleFactor);
        if (leftFlywheel != null) {
            telemetry.addData("Current L/R", "%.0f / %.0f", leftFlywheel.getVelocity(), rightFlywheel != null ? rightFlywheel.getVelocity() : 0);
        }

        telemetry.addData("--- Turret ---", "");
        if (robot.turret != null) {
            boolean inRed = ShootingZones.isInRedGoalZone(px, py);
            boolean inBlue = ShootingZones.isInBlueGoalZone(px, py);
            String zonaTexto = inRed ? "Sim - Red" : (inBlue ? "Sim - Blue" : "Nao");
            telemetry.addData("Posicao", "X: %.2f  Y: %.2f", px, py);
            telemetry.addData("Dentro da zona?", zonaTexto);
            telemetry.addData("Angle", "%.1f°", robot.turret.getMotorAngle());
            telemetry.addData("Travada (A GP2)", turretLocked ? "SIM" : "NÃO");
            telemetry.addData("--- Turret DEBUG ---", "");
            telemetry.addData("angle (°) [-180,180]", "%.2f", robot.turret.getMotorAngle());
            telemetry.addData("angle unwrapped (°)", "%.2f", robot.turret.getMotorAngleUnwrapped());
            telemetry.addData("angle wrapped (°)", "%.2f", robot.turret.getDebugCurrentAngleWrapped());
            telemetry.addData("target (°)", "%.2f", robot.turret.getDebugLastTargetDeg());
            telemetry.addData("angleToTarget raw (°)", "%.2f", robot.turret.getDebugLastAngleToTargetDeg());
            telemetry.addData("error (°)", "%.2f", robot.turret.getDebugLastError());
            telemetry.addData("power raw", "%.3f", robot.turret.getDebugLastPowerRaw());
            telemetry.addData("power out", "%.3f", robot.turret.getDebugLastPowerOut());
            telemetry.addData("limits", "[%.0f, %.0f]", robot.turret.getDebugLimitsMin(), robot.turret.getDebugLimitsMax());
            telemetry.addData("encoder raw", "%d", robot.turret.getEncoderPositionRaw());
            telemetry.addData("encoder zero", "%d", robot.turret.getEncoderZeroPosition());
        }
        if (robot.hood != null && robot.hood.isEnabled()) {
            telemetry.addData("Hood", "%.2f", robot.hood.getCurrentAngle());
        }
        telemetry.addData("--- Pose / Localizacao ---", "");
        telemetry.addData("Pose", "X: %.2f  Y: %.2f  Head: %.1f°", follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading()));
        if (kalmanFilter != null) {
            telemetry.addData("Fusao Limelight (RB GP2)", kalmanFilter.isVisionFusionEnabled() ? "ON" : "OFF");
            String fonte = kalmanFilter.isVisionFusionEnabled() && kalmanFilter.hasVision()
                    ? ("Limelight+Pinpoint (" + kalmanFilter.getValidTagCount() + " tags)")
                    : "So Pinpoint (odometria)";
            telemetry.addData("Fonte da pose", fonte);
            if (kalmanFilter.isVisionFusionEnabled() && !kalmanFilter.hasVision()) {
                telemetry.addData("Motivo (visao rejeitada)", kalmanFilter.getLastRejectionReason());
                telemetry.addData("Tags vistas pela LL", kalmanFilter.getLastFiducialIdsSeen());
            }
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
