package org.firstinspires.ftc.teamcode.drive.objects;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-oriented drive implementation for mecanum drivebase.
 * Uses IMU to maintain field-relative movement regardless of robot orientation.
 */
public class FieldOrientedDrive {
    DcMotor leftFront, leftBack, rightFront, rightBack;
    IMU imu;

    public FieldOrientedDrive(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        leftBack = hardwareMap.get(DcMotor.class, "BL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        rightBack = hardwareMap.get(DcMotor.class, "BR");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
    }

    public void movement(double lx, double ly, double rx, boolean resetIMU) {
        double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

        if (resetIMU) {
            imu.resetYaw();
        }

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double adjustedLx = ly * Math.sin(heading) + lx * Math.cos(heading);
        double adjustedLy = ly * Math.cos(heading) - lx * Math.sin(heading);

        leftFront.setPower(((adjustedLy + adjustedLx + rx) / max));
        leftBack.setPower(((adjustedLy - adjustedLx + rx) / max));
        rightFront.setPower(((adjustedLy - adjustedLx - rx) / max));
        rightBack.setPower(((adjustedLy + adjustedLx - rx) / max));
    }
}
