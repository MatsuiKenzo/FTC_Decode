package org.firstinspires.ftc.teamcode.drive.national.actuators;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.national.objects.FOD;

@TeleOp
public class MovFOD extends LinearOpMode{
    FOD fod;

    public void runOpMode(){
        fod = new FOD (hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            fod.movement(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_bumper);
        }
    }
}