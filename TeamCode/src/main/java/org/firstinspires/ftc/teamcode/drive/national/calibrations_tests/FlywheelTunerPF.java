package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

/**
 * Tuner manual de P e F do flywheel (Nacional – dois motores).
 * Y: alterna entre velocidade alta e baixa. B: muda step. D-Pad: ajusta P e F.
 */
@TeleOp(name = "Flywheel PF Tuner", group = "Tuning")
public class FlywheelTunerPF extends OpMode {

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;

    private double highVelocity = 1500;
    private double lowVelocity = 900;
    private double curTargetVelocity = highVelocity;

    private double F = 0;
    private double P = 0;

    private double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    private int stepIndex = 1;

    private boolean yPrev = false;
    private boolean bPrev = false;
    private boolean dpadLeftPrev = false;
    private boolean dpadRightPrev = false;
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;

    @Override
    public void init() {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_LEFT_MOTOR_NAME);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_RIGHT_MOTOR_NAME);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Flywheel PF Tuner (2 motores) - Init Complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean yNow = gamepad1.y;
        boolean bNow = gamepad1.b;
        boolean leftNow = gamepad1.dpad_left;
        boolean rightNow = gamepad1.dpad_right;
        boolean upNow = gamepad1.dpad_up;
        boolean downNow = gamepad1.dpad_down;

        if (yNow && !yPrev) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }
        yPrev = yNow;

        if (bNow && !bPrev) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        bPrev = bNow;

        if (leftNow && !dpadLeftPrev) {
            F += stepSizes[stepIndex];
        }
        dpadLeftPrev = leftNow;
        if (rightNow && !dpadRightPrev) {
            F -= stepSizes[stepIndex];
        }
        dpadRightPrev = rightNow;
        if (downNow && !dpadDownPrev) {
            P += stepSizes[stepIndex];
        }
        dpadDownPrev = downNow;
        if (upNow && !dpadUpPrev) {
            P -= stepSizes[stepIndex];
        }
        dpadUpPrev = upNow;

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        leftFlywheel.setVelocity(curTargetVelocity);
        rightFlywheel.setVelocity(curTargetVelocity);

        double curLeft = leftFlywheel.getVelocity();
        double curRight = rightFlywheel.getVelocity();
        double curVelocity = (curLeft + curRight) / 2.0;
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", "%.0f", curTargetVelocity);
        telemetry.addData("Current L | R", "%.1f | %.1f", curLeft, curRight);
        telemetry.addData("Current Avg", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("P", "%.4f", P);
        telemetry.addData("F", "%.4f", F);
        telemetry.addData("Step Size", "%.4f", stepSizes[stepIndex]);
        telemetry.addLine("Y=vel B=step Dpad=P/F");
        telemetry.update();
    }
}
