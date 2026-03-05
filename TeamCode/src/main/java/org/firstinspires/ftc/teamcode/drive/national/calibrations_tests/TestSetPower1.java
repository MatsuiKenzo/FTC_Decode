package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

@TeleOp(name = "TestSetPower1", group = "Test")
public class TestSetPower1 extends OpMode {

    private DcMotorEx shooterMotor;
    private DcMotorEx shooterMotor2;
    private DcMotorEx intakeMotor;
    private DcMotorEx intakeMotor2;

    @Override
    public void init() {
        try {
            shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_right");
            shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            shooterMotor = null;
        }
        try {
            shooterMotor2 = hardwareMap.get(DcMotorEx.class, "shooter_left");
            shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            shooterMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            shooterMotor2 = null;
        }
        try {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
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



        telemetry.update();
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            shooterMotor.setPower(1);
        }

        if(gamepad1.b){
            shooterMotor2.setPower(1);
        }

        if(gamepad1.x){
            intakeMotor.setPower(1);
            intakeMotor2.setPower(1);
        }

        if(gamepad1.y){
            intakeMotor.setPower(0);
            intakeMotor2.setPower(0);
        }

        telemetry.clear();
        telemetry.addLine("=== TEST SET POWER ===");
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooterMotor != null) shooterMotor.setPower(0.0);
        if (shooterMotor2 != null) shooterMotor2.setPower(0.0);
    }
}
