package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ChassiHardware extends OpMode {
    DcMotor bl, br, fl, fr;
    public void init(){
        bl = hardwareMap.get(DcMotor.class,"BL");
        br = hardwareMap.get(DcMotor.class,"BR");
        fl = hardwareMap.get(DcMotor.class,"FL");
        fr = hardwareMap.get(DcMotor.class,"FR");
    }
    public void loop(){
        if (gamepad1.a){
            bl.setPower(1);
            telemetry.addLine("BL girando");
        }
        if (gamepad1.b){
            br.setPower(1);
            telemetry.addLine("BR girando");
        }
        if(gamepad1.y){
            fl.setPower(1);
            telemetry.addLine("FL girando");
        }
        if(gamepad1.x){
            fr.setPower(1);
            telemetry.addLine("FR girando");
        }
    }
}
