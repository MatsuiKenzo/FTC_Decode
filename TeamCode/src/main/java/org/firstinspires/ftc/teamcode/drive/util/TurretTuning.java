package org.firstinspires.ftc.teamcode.drive.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.drive.national.hardware.RobotHardwareNacional;
import org.firstinspires.ftc.teamcode.drive.national.objects.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Linear Interp", group = "Tuning")
public class TurretTuning extends LinearOpMode {

    public static double targetVelocity = 1500.0;
    public static double hoodPos = 0.9;
    public static double kP = 67.0;
    public static double kI = 0.000001;
    public static double kD = 0.001;
    public static double kF = 16.0;

    private RobotHardwareNacional robot;
    private FieldOrientedDrive fod;
    private Follower follower;
    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;
    private DcMotorEx intakeMotor2;

    private final Pose startPose = new Pose(39, 80, Math.toRadians(180));
    private boolean shooterActive = false;

    private boolean aPrevGp1 = false;
    private boolean leftTriggerPrev = false;
    private boolean rightTriggerPrev = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = FtcDashboard.getInstance().getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        robot = new RobotHardwareNacional(hardwareMap, follower, false);
        fod = new FieldOrientedDrive(hardwareMap);

        try {
            intakeMotor2 = hardwareMap.get(DcMotorEx.class, "intake_2");
            intakeMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        } catch (Exception e) {
            intakeMotor2 = null;
        }

        try {
            leftFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_LEFT_MOTOR_NAME);
            rightFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_RIGHT_MOTOR_NAME);
            leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFlywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        } catch (Exception e) {
            leftFlywheel = null;
            rightFlywheel = null;
        }

        telemetry.addData("Status", "A=flywheel LT=intake RT=flap B=reset");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            robot.intake.update();

            if (robot.hood != null && robot.hood.isEnabled()) {
                robot.hood.setPositionOverride(hoodPos);
            }
            robot.updateWithoutShooter();

            fod.movement(
                    -gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    gamepad1.left_bumper
            );

            boolean leftTriggerNow = gamepad1.left_trigger > 0.1;
            robot.intake.toggleIntake(leftTriggerNow && !leftTriggerPrev);
            leftTriggerPrev = leftTriggerNow;
            if (robot.intake.isIntakeActive()) {
                robot.intake.setPower(1.0);
                if (intakeMotor2 != null) intakeMotor2.setPower(1.0);
            } else {
                if (intakeMotor2 != null) intakeMotor2.setPower(0.0);
            }

            boolean rightTriggerNow = gamepad1.right_trigger > 0.1;
            robot.intake.shoot(rightTriggerNow && !rightTriggerPrev);
            rightTriggerPrev = rightTriggerNow;

            if (gamepad1.b) {
                follower.setPose(startPose);
            }

            if (leftFlywheel != null && rightFlywheel != null) {
                PIDFCoefficients pidf = new PIDFCoefficients(kP, kI, kD, kF);
                leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
                rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

                boolean aNow = gamepad1.a;
                if (aNow && !aPrevGp1) {
                    shooterActive = !shooterActive;
                }
                aPrevGp1 = aNow;

                if (shooterActive) {
                    leftFlywheel.setVelocity(targetVelocity);
                    rightFlywheel.setVelocity(targetVelocity);
                } else {
                    leftFlywheel.setVelocity(0);
                    rightFlywheel.setVelocity(0);
                }
            }

            double tpr = ConstantsConf.Shooter.TICKS_PER_REVOLUTION;
            double targetRPM = tpr > 0 ? targetVelocity / tpr * 60.0 : 0;
            double curVelocity = 0;
            double curRPM = 0;
            double velocityError = 0;
            double errorRPM = 0;
            if (leftFlywheel != null && rightFlywheel != null) {
                double vL = leftFlywheel.getVelocity();
                double vR = rightFlywheel.getVelocity();
                curVelocity = (vL + vR) / 2.0;
                curRPM = tpr > 0 ? curVelocity / tpr * 60.0 : 0;
                velocityError = targetVelocity - curVelocity;
                errorRPM = tpr > 0 ? velocityError / tpr * 60.0 : 0;
            }

            telemetry.addData("targetVelocity", "%.0f", targetVelocity);
            telemetry.addData("targetRPM", "%.0f", targetRPM);
            telemetry.addData("curVelocity", "%.0f", curVelocity);
            telemetry.addData("curRPM", "%.0f", curRPM);
            telemetry.addData("velocityError", "%.0f", velocityError);
            telemetry.addData("errorRPM", "%.0f", errorRPM);
            telemetry.addData("kP", "%.4f", kP);
            telemetry.addData("kI", "%.6f", kI);
            telemetry.addData("kD", "%.4f", kD);
            telemetry.addData("kF", "%.4f", kF);
            telemetry.addData("hoodPos", "%.3f", hoodPos);
            telemetry.addData("Shooter (A)", shooterActive ? "ON" : "OFF");
            telemetry.addData("Intake (LT)", robot.intake.isIntakeActive() ? "ON" : "OFF");
            if (leftFlywheel != null && rightFlywheel != null) {
                double vL = leftFlywheel.getVelocity();
                double vR = rightFlywheel.getVelocity();
                double rpmL = tpr > 0 ? vL / tpr * 60.0 : 0;
                double rpmR = tpr > 0 ? vR / tpr * 60.0 : 0;
                telemetry.addData("RPM L | R", "%.0f | %.0f", rpmL, rpmR);
            }
            if (robot.hood != null && robot.hood.isEnabled()) {
                telemetry.addData("Hood (servo)", "%.3f", robot.hood.getCurrentAngle());
            }
            telemetry.update();
        }

        if (intakeMotor2 != null) intakeMotor2.setPower(0.0);
        if (leftFlywheel != null) leftFlywheel.setVelocity(0);
        if (rightFlywheel != null) rightFlywheel.setVelocity(0);
        robot.stop();
    }
}
