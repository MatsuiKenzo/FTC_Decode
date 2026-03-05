package org.firstinspires.ftc.teamcode.drive.national;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.national.subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp
public class HeitorShooterTuning extends LinearOpMode {
    public static double hoodPos = 0;
    public static int flywheelVelocity = 0;
    public static double p = 0;
    public static double i = 0;
    public static double d = 0;
    public static double ff = 0;





    @Override
    public void runOpMode() throws InterruptedException {
        Turret turret = new Turret(hardwareMap);
        Follower follower = Constants.createFollower(hardwareMap);
        turret.setSide(Turret.SIDES.BLUE);
        follower.setPose(new Pose(13.25,17.7/2,0));
        this.telemetry = FtcDashboard.getInstance().getTelemetry();
        waitForStart();

        while (!isStopRequested()){
            follower.update();
            turret.setBotPose(follower.getPose());
            turret.periodic();
            turret.setPIDf(p,i,d,ff);
            turret.setHoodPosition(hoodPos);
            turret.setShooterVelocity(flywheelVelocity);
            telemetry.addData("target velocity: ", flywheelVelocity);
            telemetry.addData("Angle:", turret.getTurretAngleDegrees());
            telemetry.addData("current velocity: ", turret.getShooterVelocity());
            telemetry.addData("distance: ", turret.getDistanceToGoal());
            telemetry.update();


        }
    }
}
