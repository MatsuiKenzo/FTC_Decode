package org.firstinspires.ftc.teamcode.drive.actuators;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

@TeleOp(name = "Flywheel PIDF Tester", group = "Tuning")
public class FlywheelTester extends OpMode {

    public DcMotorEx flywheelMotor;

    double curTargetVelocity = 0;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "RMTa");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(ConstantsConf.Shooter.KP, ConstantsConf.Shooter.KI, ConstantsConf.Shooter.KD, ConstantsConf.Shooter.KF);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
    }

    @Override
    public void loop() {
        if (Math.abs(gamepad1.right_stick_y) > 0.1) {
            curTargetVelocity += gamepad1.right_stick_y * 50;
            curTargetVelocity = Math.max(0, Math.min(6000, curTargetVelocity));
        }

        if (gamepad1.a) {
            flywheelMotor.setVelocity(curTargetVelocity);
        }


        double curVelocity = flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity: ", curTargetVelocity);
        telemetry.addData("Current Velocity: ", "%.2f", curVelocity);
        telemetry.addData("Error: ","%,2f", error);
        telemetry.update();
    }


}
