package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.national.hardware.RobotHardwareNacional;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Tuner do PID da turret — um controle só.
 * A = alternar alvo entre -90° e 90°. B = ciclo ganho (kP/kI/kD). D-pad U/D = ajustar ganho. Stick X = manual. Y = zerar 0°.
 */
@TeleOp(name = "Turret PID Tuner", group = "Tuning")
public class TurretPIDTuner extends OpMode {

    private Follower follower;
    private RobotHardwareNacional robot;

    private static final Pose FIXED_POSE = new Pose(80, 80, Math.toRadians(180));
    private static final double POS_A_DEG = -90.0;
    private static final double POS_B_DEG = 90.0;

    /** true = alvo 90°, false = alvo -90° */
    private boolean targetIs90 = false;
    private double targetAngleDeg = POS_A_DEG;

    private double kP = 0.06;
    private double kI = 0.0;
    private double kD = 0.005;

    private static final double STEP_KP = 0.005;
    private static final double STEP_KI = 0.0005;
    private static final double STEP_KD = 0.001;

    private int selectedGain = 0;
    private boolean aPrev = false;
    private boolean bPrev = false;
    private boolean dpadUpPrev = false;
    private boolean dpadDownPrev = false;
    private boolean yPrev = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(FIXED_POSE);
        robot = new RobotHardwareNacional(hardwareMap, follower, true);

        if (robot.turret != null) {
            robot.turret.resetAngle(0.0);
            robot.turret.lockAngle(targetAngleDeg);
            robot.turret.setPID(kP, kI, kD);
        }

        telemetry.addLine("Turret PID Tuner (1 controle)");
        telemetry.addLine("A = alvo -90° / 90° | B = ciclo ganho | D-pad U/D = ajustar");
        telemetry.addLine("Stick X = manual | Y = zerar 0°");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (robot.turret == null) {
            telemetry.addLine("Turret nao disponivel.");
            telemetry.update();
            return;
        }

        // ----- A: alternar alvo entre -90° e 90°
        boolean aNow = gamepad1.a;
        if (aNow && !aPrev) {
            targetIs90 = !targetIs90;
            targetAngleDeg = targetIs90 ? POS_B_DEG : POS_A_DEG;
        }
        aPrev = aNow;

        robot.turret.lockAngle(targetAngleDeg);

        // ----- B: ciclo do ganho (kP -> kI -> kD -> kP)
        boolean bNow = gamepad1.b;
        if (bNow && !bPrev) {
            selectedGain = (selectedGain + 1) % 3;
        }
        bPrev = bNow;

        // ----- D-pad U/D: ajustar ganho selecionado
        boolean upNow = gamepad1.dpad_up;
        boolean downNow = gamepad1.dpad_down;
        if (upNow && !dpadUpPrev) adjustGain(1);
        if (downNow && !dpadDownPrev) adjustGain(-1);
        dpadUpPrev = upNow;
        dpadDownPrev = downNow;

        robot.turret.setPID(kP, kI, kD);

        // ----- Y: zerar ângulo em 0°
        boolean yNow = gamepad1.y;
        if (yNow && !yPrev) {
            robot.turret.resetAngle(0.0);
            targetIs90 = false;
            targetAngleDeg = POS_A_DEG;
            robot.turret.lockAngle(targetAngleDeg);
        }
        yPrev = yNow;

        // ----- Stick X = manual; senão PID
        double manualPower = -gamepad1.left_stick_x;
        if (Math.abs(manualPower) > 0.08) {
            robot.turret.setManualPower(manualPower);
        } else {
            robot.turret.update();
        }

        // ----- Telemetria
        telemetry.clear();
        telemetry.addLine("=== TURRET PID TUNER ===");
        telemetry.addData("Alvo (°)", "%.1f (A = trocar)", targetAngleDeg);
        telemetry.addData("angle (°)", "%.2f", robot.turret.getMotorAngle());
        telemetry.addData("angle unwrapped (°)", "%.2f", robot.turret.getMotorAngleUnwrapped());
        telemetry.addData("target (°)", "%.2f", robot.turret.getDebugLastTargetDeg());
        telemetry.addData("error (°)", "%.2f", robot.turret.getDebugLastError());
        telemetry.addData("power raw", "%.3f", robot.turret.getDebugLastPowerRaw());
        telemetry.addData("power out", "%.3f", robot.turret.getDebugLastPowerOut());
        telemetry.addLine("--- Ganhos (B = ciclo, D-pad U/D = ajustar) ---");
        String kPLabel = selectedGain == 0 ? ">> kP" : "   kP";
        String kILabel = selectedGain == 1 ? ">> kI" : "   kI";
        String kDLabel = selectedGain == 2 ? ">> kD" : "   kD";
        telemetry.addData(kPLabel, "%.4f", kP);
        telemetry.addData(kILabel, "%.4f", kI);
        telemetry.addData(kDLabel, "%.4f", kD);
        telemetry.addLine("A=-90/90 | B=ciclo | D-pad=ajustar | Stick X=manual | Y=zerar");
        telemetry.update();
    }

    private void adjustGain(int direction) {
        switch (selectedGain) {
            case 0:
                kP = Math.max(0.0, kP + direction * STEP_KP);
                break;
            case 1:
                kI = Math.max(0.0, kI + direction * STEP_KI);
                break;
            case 2:
                kD = Math.max(0.0, kD + direction * STEP_KD);
                break;
        }
    }

    @Override
    public void stop() {
        if (robot != null && robot.turret != null) {
            robot.turret.stopRotation();
        }
    }
}
