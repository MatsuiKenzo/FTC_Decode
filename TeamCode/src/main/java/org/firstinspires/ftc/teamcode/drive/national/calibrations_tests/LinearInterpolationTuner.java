package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.national.hardware.RobotHardwareNacional;
import org.firstinspires.ftc.teamcode.drive.national.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Tuner da interpolação linear distância → RPM (Nacional).
 * Shooter como nas classes de calibração: motores diretos com setVelocity(0) ao desligar (para de girar).
 * A = ligar/desligar shooter na velocidade definida. D-Pad Up/Down = ajustar velocidade, X = scale (10/100/1000).
 * Intake e flap como Flap Intake Tester: LT = intake, RT = flap.
 *
 * Gamepad 1:
 *   - Stick: drive | LB: reset IMU | B: reset pose | Y: recalibrar alvo
 *   - LT: toggle intake | RT: flap | A: toggle shooter | D-Pad U/D: velocidade | X: scale
 *
 * Gamepad 2:
 *   - Stick esquerdo Y: potência do intake (0 a 1)
 */
@TeleOp(name = "Linear Interpolation Tuner", group = "Tuning")
public class LinearInterpolationTuner extends LinearOpMode {

    private RobotHardwareNacional robot;
    private FieldOrientedDrive fod;
    private Follower follower;

    /** Shooter: controle direto dos motores (igual Flap Intake Tester) para não engasgar e zero parar. */
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;

    /** Motor opcional intake_2 (igual Flap Intake Tester). */
    private DcMotorEx intakeMotor2;

    private final Pose startPose = new Pose(39, 80, Math.toRadians(180));
    private static final double TARGET_X = 6.0;
    private static final double TARGET_Y = 138.0;

    /** Velocidade alvo em ticks/s (como Flap Intake Tester). A = liga/desliga nessa velocidade. */
    private double curTargetVelocity = 1500.0;
    private double scaleFactor = 50.0;
    private int scaleMode = 0; // 0=50, 1=100, 2=1000
    private boolean shooterActive = false;

    private double intakePower = ConstantsConf.Intake.INTAKE_POWER;

    private boolean aPrevGp1 = false;
    private boolean yPrevGp1 = false;
    private boolean xPrevGp1 = false;
    private boolean leftTriggerPrev = false;
    private boolean rightTriggerPrev = false;

    @Override
    public void runOpMode() {
        // Inicialização do PedroPathing e Hardware Nacional
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        // Se NÃO tiver turret conectada, use: new RobotHardwareNacional(hardwareMap, follower, false)
        robot = new RobotHardwareNacional(hardwareMap, follower, false);
        fod = new FieldOrientedDrive(hardwareMap);

        // Intake_2 opcional (igual Flap Intake Tester)
        try {
            intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake_2");
            intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            intakeMotor2 = null;
        }

        // Shooter: motores diretos como nas classes de calibração (para não engasgar; zero = para)
        try {
            leftFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_LEFT_MOTOR_NAME);
            rightFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_RIGHT_MOTOR_NAME);
            leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
            PIDFCoefficients pidf = new PIDFCoefficients(
                    ConstantsConf.Shooter.KP,
                    ConstantsConf.Shooter.KI,
                    ConstantsConf.Shooter.KD,
                    ConstantsConf.Shooter.KF
            );
            leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
            rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        } catch (Exception e) {
            leftFlywheel = null;
            rightFlywheel = null;
        }

        telemetry.addData("Status", "Linear Interpolation Tuner (distância → RPM)");
        telemetry.addData("Info", "GP1: A=shooter D-Pad=vel X=scale LT=intake RT=flap | GP2: L stick=potência intake");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            robot.intake.update(); // só flap state machine; shooter é controle direto abaixo

            // Gamepad 1: Movimentação (Field Oriented)
            fod.movement(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_bumper
            );

            // Intake toggle no LT (igual Flap Intake Tester)
            boolean leftTriggerNow = gamepad1.left_trigger > 0.1;
            robot.intake.toggleIntake(leftTriggerNow && !leftTriggerPrev);
            leftTriggerPrev = leftTriggerNow;

            // Potência do intake no GP2 stick esquerdo Y (0 a 1)
            if (Math.abs(gamepad2.left_stick_y) > 0.05) {
                intakePower += gamepad2.left_stick_y * 0.02;
                intakePower = Math.max(0.0, Math.min(1.0, intakePower));
            }
            if (robot.intake.isIntakeActive()) {
                robot.intake.setPower(intakePower);
                if (intakeMotor2 != null) intakeMotor2.setPower(intakePower);
            } else {
                if (intakeMotor2 != null) intakeMotor2.setPower(0.0);
            }

            // Shoot (Flap) no RT — ciclo alinhar → 2s → voltar (igual Flap Intake Tester)
            boolean rightTriggerNow = gamepad1.right_trigger > 0.1;
            robot.intake.shoot(rightTriggerNow && !rightTriggerPrev);
            rightTriggerPrev = rightTriggerNow;

            // Reset Pose no B
            if (gamepad1.b) {
                follower.setPose(startPose);
            }

            // Recalibrar alvo no Y (direção = heading + turret se tiver turret, senão só heading)
            boolean yNow = gamepad1.y;
            if (yNow && !yPrevGp1) {
                Pose robotPose = follower.getPose();
                double dx = TARGET_X - robotPose.getX();
                double dy = TARGET_Y - robotPose.getY();
                double dist = Math.hypot(dx, dy);
                double headingRad = robotPose.getHeading();
                double absoluteAngleRad = headingRad;
                if (robot.turret != null) {
                    absoluteAngleRad = headingRad + Math.toRadians(robot.turret.getMotorAngle());
                }
                double newTargetX = robotPose.getX() + dist * Math.cos(absoluteAngleRad);
                double newTargetY = robotPose.getY() + dist * Math.sin(absoluteAngleRad);
                robot.shooter.setTargetPosition(newTargetX, newTargetY);
            }
            yPrevGp1 = yNow;

            // Shooter: A = ligar/desligar; D-Pad Up/Down = velocidade; X = scale (igual Flap Intake Tester)
            if (leftFlywheel != null && rightFlywheel != null) {
                if (gamepad1.dpad_up) {
                    curTargetVelocity += scaleFactor;
                    curTargetVelocity = Math.min(6000, curTargetVelocity);
                }
                if (gamepad1.dpad_down) {
                    curTargetVelocity -= scaleFactor;
                    curTargetVelocity = Math.max(0, curTargetVelocity);
                }
                boolean xNow = gamepad1.x;
                if (xNow && !xPrevGp1) {
                    scaleMode = (scaleMode + 1) % 3;
                    switch (scaleMode) {
                        case 0: scaleFactor = 10.0; break;
                        case 1: scaleFactor = 100.0; break;
                        case 2: scaleFactor = 1000.0; break;
                    }
                }
                xPrevGp1 = xNow;

                boolean aNow = gamepad1.a;
                if (aNow && !aPrevGp1) {
                    shooterActive = !shooterActive;
                }
                aPrevGp1 = aNow;

                if (shooterActive) {
                    leftFlywheel.setVelocity(curTargetVelocity);
                    rightFlywheel.setVelocity(curTargetVelocity);
                } else {
                    leftFlywheel.setVelocity(0);
                    rightFlywheel.setVelocity(0);
                }
            }

            // Telemetria para calibração
            Pose currentPose = follower.getPose();
            double dx = TARGET_X - currentPose.getX();
            double dy = TARGET_Y - currentPose.getY();
            double distPol = Math.hypot(dx, dy);

            telemetry.addData("--- Odometria (Nacional) ---", "");
            telemetry.addData("Pose X | Y", "%.1f | %.1f", currentPose.getX(), currentPose.getY());
            telemetry.addData("Heading (°)", "%.1f", Math.toDegrees(currentPose.getHeading()));
            telemetry.addLine();
            telemetry.addData(">>> INTERPOLAÇÃO (distância → RPM) <<<", "");
            telemetry.addData("Distância ao Alvo (pol)", "%.1f", distPol);
            double tpr = ConstantsConf.Shooter.TICKS_PER_REVOLUTION;
            double targetRPM = tpr > 0 ? curTargetVelocity / tpr * 60.0 : 0;
            telemetry.addData("RPM Alvo (D-Pad)", "%.0f", targetRPM);
            telemetry.addData("Ponto para LUT", "%.1f pol, %.0f RPM → ConstantsConf.Shooter", distPol, targetRPM);
            telemetry.addLine();
            telemetry.addData("--- Shooter (igual calibração: A=liga) ---", "");
            telemetry.addData("Shooter (A)", shooterActive ? "LIGADO" : "DESLIGADO");
            telemetry.addData("Target (ticks/s)", "%.0f  Scale (X)=%.0f", curTargetVelocity, scaleFactor);
            if (leftFlywheel != null && rightFlywheel != null) {
                double vL = leftFlywheel.getVelocity();
                double vR = rightFlywheel.getVelocity();
                double rpmL = tpr > 0 ? vL / tpr * 60.0 : 0;
                double rpmR = tpr > 0 ? vR / tpr * 60.0 : 0;
                telemetry.addData("Current L | R (RPM)", "%.0f | %.0f", rpmL, rpmR);
            }
            telemetry.addLine();
            telemetry.addData("--- Intake (igual Flap Intake Tester) ---", "");
            telemetry.addData("Intake (LT)", robot.intake.isIntakeActive() ? "ON" : "OFF");
            telemetry.addData("Potência intake (GP2 L stick Y)", "%.2f", intakePower);
            telemetry.addData("Intake_2", intakeMotor2 != null ? "conectado" : "não configurado");
            telemetry.addLine();
            telemetry.addData("Flap (RT)", "alinhar → 2s → voltar");
            telemetry.update();
        }

        if (intakeMotor2 != null) intakeMotor2.setPower(0.0);
        if (leftFlywheel != null) leftFlywheel.setVelocity(0);
        if (rightFlywheel != null) rightFlywheel.setVelocity(0);
        robot.stop();
    }
}
