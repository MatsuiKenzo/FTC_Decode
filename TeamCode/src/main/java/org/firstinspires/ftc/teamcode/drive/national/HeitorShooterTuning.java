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
        Turret turret;
        try {
            turret = new Turret(hardwareMap);
        } catch (Exception e) {
            telemetry.addLine("ERRO: Falha ao inicializar Turret: " + e.getMessage());
            telemetry.update();
            return;
        }

        Follower follower = null;
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setPose(new Pose(13.25, 17.7 / 2, 0));
        } catch (Exception e) {
            telemetry.addLine("AVISO: Follower não disponível (drive/pinpoint). Usando pose fixa.");
        }

        turret.setSide(Turret.SIDES.BLUE);
        Pose staticPose = new Pose(13.25, 17.7 / 2, 0);
        this.telemetry = FtcDashboard.getInstance().getTelemetry();
        waitForStart();

        while (!isStopRequested()) {
            if (follower != null) {
                follower.update();
                turret.setBotPose(follower.getPose());
            } else {
                turret.setBotPose(staticPose);
            }
            turret.periodic();
            turret.setPIDf(p, i, d, ff);
            turret.setHoodPosition(hoodPos);
            turret.setShooterVelocity(flywheelVelocity);
            telemetry.addData("target velocity", flywheelVelocity);
            telemetry.addData("Angle (°)", "%.2f", turret.getTurretAngleDegrees());
            telemetry.addData("current velocity", "%.0f", turret.getShooterVelocity());
            telemetry.addData("distance (in)", "%.2f", turret.getDistanceToGoal());
            telemetry.update();
        }
    }
}
