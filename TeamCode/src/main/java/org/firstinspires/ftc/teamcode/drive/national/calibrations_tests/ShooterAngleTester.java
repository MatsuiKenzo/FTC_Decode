package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Teste do shooter com ajuste de ângulo (hood/servo).
 *
 * Controles:
 * - D-pad Cima/Baixo: ajuste do ângulo do servo (1.0 = mínimo, 0.5 = máximo)
 * - D-pad Esquerda/Direita: ajuste da potência/velocidade do shooter
 * - A: liga os motores na velocidade atual (segure para girar)
 *
 * Init: servo em 1.0 (ângulo mínimo). Faixa do servo: 1.0 até 0.5 (ângulo máximo).
 * Os dois motores do shooter estão em FORWARD.
 */
@TeleOp(name = "Shooter Angle Tester", group = "Tuning")
public class ShooterAngleTester extends OpMode {

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private Servo angleServo;

    /** Posição do servo: 1.0 = ângulo mínimo, 0.5 = ângulo máximo. */
    private static final double SERVO_POS_MIN = 1.0;
    private static final double SERVO_POS_MAX = 0.5;
    private static final double SERVO_STEP = 0.1;

    private double servoPosition = SERVO_POS_MIN;
    private double targetVelocity = 0.0;
    private static final double VELOCITY_STEP = 100.0;
    private static final double VELOCITY_MIN = 0.0;
    private static final double VELOCITY_MAX = 5000.0;

    private boolean dpadUpPrev, dpadDownPrev, dpadLeftPrev, dpadRightPrev;

    @Override
    public void init() {
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
            angleServo = hardwareMap.get(Servo.class, ConstantsConf.Nacional.HOOD_SERVO_NAME);
            servoPosition = SERVO_POS_MIN;
            angleServo.setPosition(servoPosition);
        } catch (Exception e) {
            angleServo = null;
        }

        dpadUpPrev = dpadDownPrev = dpadLeftPrev = dpadRightPrev = false;
        telemetry.addLine("Shooter Angle Tester");
        telemetry.addLine("D-pad U/D = angulo | D-pad L/R = potencia | A = girar");
        telemetry.addLine("Servo: 1.0 = min, 0.5 = max");
        telemetry.update();
    }

    @Override
    public void loop() {
        // D-pad Up/Down: ângulo (servo)
        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        if (dpadUp && !dpadUpPrev && angleServo != null) {
            servoPosition -= SERVO_STEP;
            servoPosition = Range.clip(servoPosition, SERVO_POS_MAX, SERVO_POS_MIN);
            angleServo.setPosition(servoPosition);
        }
        if (dpadDown && !dpadDownPrev && angleServo != null) {
            servoPosition += SERVO_STEP;
            servoPosition = Range.clip(servoPosition, SERVO_POS_MAX, SERVO_POS_MIN);
            angleServo.setPosition(servoPosition);
        }
        dpadUpPrev = dpadUp;
        dpadDownPrev = dpadDown;

        // D-pad Left/Right: potência (velocidade)
        boolean dpadLeft = gamepad1.dpad_left;
        boolean dpadRight = gamepad1.dpad_right;
        if (dpadRight && !dpadRightPrev) {
            targetVelocity += VELOCITY_STEP;
            targetVelocity = Range.clip(targetVelocity, VELOCITY_MIN, VELOCITY_MAX);
        }
        if (dpadLeft && !dpadLeftPrev) {
            targetVelocity -= VELOCITY_STEP;
            targetVelocity = Range.clip(targetVelocity, VELOCITY_MIN, VELOCITY_MAX);
        }
        dpadLeftPrev = dpadLeft;
        dpadRightPrev = dpadRight;

        // A: girar os dois motores na velocidade atual
        boolean run = gamepad1.a;
        if (leftFlywheel != null && rightFlywheel != null) {
            if (run) {
                leftFlywheel.setVelocity(targetVelocity);
                rightFlywheel.setVelocity(targetVelocity);
            } else {
                leftFlywheel.setVelocity(0);
                rightFlywheel.setVelocity(0);
            }
        }

        // Telemetria
        telemetry.addData("--- Shooter Angle Tester ---", "");
        telemetry.addData("Servo (angulo)", "%.3f  (min=1.0, max=0.5)", servoPosition);
        telemetry.addData("Velocidade alvo", "%.0f ticks/s", targetVelocity);
        if (leftFlywheel != null && rightFlywheel != null) {
            double vL = leftFlywheel.getVelocity();
            double vR = rightFlywheel.getVelocity();
            telemetry.addData("Vel L / R", "%.0f / %.0f", vL, vR);
        }
        telemetry.addLine("D-pad U/D=angulo  L/R=potencia  A=girar");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (leftFlywheel != null) leftFlywheel.setVelocity(0);
        if (rightFlywheel != null) rightFlywheel.setVelocity(0);
    }
}
