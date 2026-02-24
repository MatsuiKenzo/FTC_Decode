package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

@TeleOp(name = "Flywheel PIDF Tester", group = "Tuning")
public class FlywheelTester extends OpMode {

    public DcMotorEx leftFlywheel;
    public DcMotorEx rightFlywheel;

    double curTargetVelocity = 0;

    @Override
    public void init() {
        leftFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_LEFT_MOTOR_NAME);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_RIGHT_MOTOR_NAME);

        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(ConstantsConf.Shooter.KP, ConstantsConf.Shooter.KI, ConstantsConf.Shooter.KD, ConstantsConf.Shooter.KF);
        leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        if (Math.abs(gamepad1.right_stick_y) > 0.1) {
            curTargetVelocity += gamepad1.right_stick_y * 50;
            curTargetVelocity = Math.max(0, Math.min(6000, curTargetVelocity));
        }

        if (gamepad1.a) {
            leftFlywheel.setVelocity(curTargetVelocity);
            rightFlywheel.setVelocity(curTargetVelocity);
        }

        double curVelocityLeft = leftFlywheel.getVelocity();
        double curVelocityRight = rightFlywheel.getVelocity();
        double curVelocityAvg = (curVelocityLeft + curVelocityRight) / 2.0;
        double error = curTargetVelocity - curVelocityAvg;

        telemetry.addData("Target Velocity: ", curTargetVelocity);
        telemetry.addData("Current Left: ", "%.2f", curVelocityLeft);
        telemetry.addData("Current Right: ", "%.2f", curVelocityRight);
        telemetry.addData("Current Avg: ", "%.2f", curVelocityAvg);
        telemetry.addData("Error: ", "%.2f", error);
        telemetry.update();
    }


}
