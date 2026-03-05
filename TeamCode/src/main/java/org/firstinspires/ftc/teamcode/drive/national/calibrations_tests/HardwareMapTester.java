package org.firstinspires.ftc.teamcode.drive.national.calibrations_tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Testa todos os motores e servos do HardwareMap (Control Hub + Expansion Hub).
 * Cada dispositivo é mapeado a um botão: ao clicar, o motor gira ou o servo move um pouco.
 *
 * GP1 = Motores (A, B, X, Y, Dpad U/D/L/R, LB, RB, LT, RT)
 * GP2 = Servos normais + CR Servos (mesmos botões)
 *
 * Nomes usados vêm da programação (ConstantsConf + drive FL/BL/FR/BR).
 */
@TeleOp(name = "HardwareMap Tester", group = "Test")
public class HardwareMapTester extends OpMode {

    private static final double MOTOR_POWER = 0.25;
    private static final double MOTOR_BURST_SEC = 0.4;
    private static final double SERVO_DELTA = 0.05;
    private static final double CRSERVO_POWER = 0.3;
    private static final double CRSERVO_BURST_SEC = 0.4;

    private static class MotorEntry {
        final String name;
        final DcMotor motor;
        MotorEntry(String name, DcMotor motor) { this.name = name; this.motor = motor; }
    }

    private static class ServoEntry {
        final String name;
        final boolean isCR;
        final Servo servo;       // usado se isCR == false
        final CRServo crServo;   // usado se isCR == true
        double lastPosition = 0.5;
        ServoEntry(String name, Servo s) { this.name = name; isCR = false; servo = s; crServo = null; }
        ServoEntry(String name, CRServo s) { this.name = name; isCR = true; servo = null; crServo = s; }
    }

    private final List<MotorEntry> motors = new ArrayList<>();
    private final List<ServoEntry> servos = new ArrayList<>();
    private final ElapsedTime motorBurstTimer = new ElapsedTime();
    private final ElapsedTime crServoBurstTimer = new ElapsedTime();
    private int motorBurstIndex = -1;
    private int crServoBurstIndex = -1;

    /** Botões GP1 para motores: A=0, B=1, X=2, Y=3, DpadU=4, DpadD=5, DpadL=6, DpadR=7, LB=8, RB=9, LT=10, RT=11 */
    private static final int MAX_MOTOR_BUTTONS = 12;
    /** Botões GP2 para servos: mesmo mapeamento */
    private static final int MAX_SERVO_BUTTONS = 12;

    private boolean gp1APrev, gp1BPrev, gp1XPrev, gp1YPrev;
    private boolean gp1DpadUPrev, gp1DpadDPrev, gp1DpadLPrev, gp1DpadRPrev;
    private boolean gp1LBPrev, gp1RBPrev, gp1LTPrev, gp1RTPrev;
    private boolean gp2APrev, gp2BPrev, gp2XPrev, gp2YPrev;
    private boolean gp2DpadUPrev, gp2DpadDPrev, gp2DpadLPrev, gp2DpadRPrev;
    private boolean gp2LBPrev, gp2RBPrev, gp2LTPrev, gp2RTPrev;

    @Override
    public void init() {
        collectMotors();
        collectServos();
        telemetry.addLine("=== HardwareMap Tester ===");
        telemetry.addData("Motores encontrados", motors.size());
        telemetry.addData("Servos encontrados", servos.size());
        telemetry.addLine("GP1: motores | GP2: servos");
        telemetry.addLine("Cada botão = um dispositivo (ver lista abaixo)");
        telemetry.update();
    }

    /** Coleta todos os motores do hardwareMap usando os nomes da programação + iteração se disponível. */
    private void collectMotors() {
        LinkedHashMap<String, DcMotor> byName = new LinkedHashMap<>();
        // 1) Nomes usados no código (ConstantsConf + drive)
        String[] motorNames = {
                "FL", "BL", "FR", "BR",
                ConstantsConf.Intake.INTAKE_MOTOR_NAME,
                "intake_2",
                ConstantsConf.Nacional.SHOOTER_LEFT_MOTOR_NAME,
                ConstantsConf.Nacional.SHOOTER_RIGHT_MOTOR_NAME,
                ConstantsConf.Nacional.TURRET_ENCODER_MOTOR_NAME,
                ConstantsConf.Shooter.FLYWHEEL_MOTOR_NAME,
                ConstantsConf.Turret.TURRET_MOTOR_NAME,
                ConstantsConf.Intake.INDEXER_MOTOR_NAME
        };
        for (String name : motorNames) {
            if (byName.containsKey(name)) continue;
            try {
                DcMotor m = hardwareMap.get(DcMotor.class, name);
                byName.put(name, m);
            } catch (Exception ignored) { }
        }
        // 2) Qualquer outro motor no config (Control Hub + Expansion Hub)
        try {
            for (Map.Entry<String, DcMotor> e : hardwareMap.dcMotor.entrySet()) {
                if (!byName.containsKey(e.getKey())) {
                    byName.put(e.getKey(), e.getValue());
                }
            }
        } catch (Exception ignored) { }
        for (Map.Entry<String, DcMotor> e : byName.entrySet()) {
            motors.add(new MotorEntry(e.getKey(), e.getValue()));
        }
    }

    /** Coleta todos os servos (Servo + CRServo) do hardwareMap. */
    private void collectServos() {
        LinkedHashMap<String, ServoEntry> byName = new LinkedHashMap<>();
        // 1) Nomes da programação
        String[] servoNames = {
                ConstantsConf.Intake.FLAP_SERVO_NAME,
                ConstantsConf.Intake.FLAP2_SERVO_NAME,
                ConstantsConf.Nacional.HOOD_SERVO_NAME,
                ConstantsConf.Nacional.TURRET_LEFT_SERVO_NAME,
                ConstantsConf.Nacional.TURRET_RIGHT_SERVO_NAME
        };
        for (String name : servoNames) {
            if (byName.containsKey(name)) continue;
            try {
                Servo s = hardwareMap.get(Servo.class, name);
                byName.put(name, new ServoEntry(name, s));
            } catch (Exception ignored) { }
            try {
                CRServo s = hardwareMap.get(CRServo.class, name);
                byName.put(name, new ServoEntry(name, s));
            } catch (Exception ignored) { }
        }
        // 2) Outros servos no config (Control Hub + Expansion Hub)
        try {
            for (Map.Entry<String, Servo> e : hardwareMap.servo.entrySet()) {
                if (!byName.containsKey(e.getKey())) {
                    byName.put(e.getKey(), new ServoEntry(e.getKey(), e.getValue()));
                }
            }
        } catch (Exception ignored) { }
        try {
            for (Map.Entry<String, CRServo> e : hardwareMap.crservo.entrySet()) {
                if (!byName.containsKey(e.getKey())) {
                    byName.put(e.getKey(), new ServoEntry(e.getKey(), e.getValue()));
                }
            }
        } catch (Exception ignored) { }
        servos.addAll(byName.values());
    }

    @Override
    public void loop() {
        // Parar motor após o burst
        if (motorBurstIndex >= 0) {
            if (motorBurstTimer.seconds() >= MOTOR_BURST_SEC) {
                motors.get(motorBurstIndex).motor.setPower(0);
                motorBurstIndex = -1;
            }
        }
        if (crServoBurstIndex >= 0) {
            if (crServoBurstTimer.seconds() >= CRSERVO_BURST_SEC) {
                servos.get(crServoBurstIndex).crServo.setPower(0);
                crServoBurstIndex = -1;
            }
        }

        // GP1: motores (edge trigger)
        int mi = getMotorButtonIndex(gamepad1.a, gamepad1.b, gamepad1.x, gamepad1.y,
                gamepad1.dpad_up, gamepad1.dpad_down, gamepad1.dpad_left, gamepad1.dpad_right,
                gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.left_trigger > 0.5, gamepad1.right_trigger > 0.5,
                gp1APrev, gp1BPrev, gp1XPrev, gp1YPrev, gp1DpadUPrev, gp1DpadDPrev, gp1DpadLPrev, gp1DpadRPrev,
                gp1LBPrev, gp1RBPrev, gp1LTPrev, gp1RTPrev);
        gp1APrev = gamepad1.a; gp1BPrev = gamepad1.b; gp1XPrev = gamepad1.x; gp1YPrev = gamepad1.y;
        gp1DpadUPrev = gamepad1.dpad_up; gp1DpadDPrev = gamepad1.dpad_down;
        gp1DpadLPrev = gamepad1.dpad_left; gp1DpadRPrev = gamepad1.dpad_right;
        gp1LBPrev = gamepad1.left_bumper; gp1RBPrev = gamepad1.right_bumper;
        gp1LTPrev = gamepad1.left_trigger > 0.5; gp1RTPrev = gamepad1.right_trigger > 0.5;

        if (mi >= 0 && mi < motors.size() && motorBurstIndex < 0) {
            MotorEntry e = motors.get(mi);
            e.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            e.motor.setPower(MOTOR_POWER);
            motorBurstTimer.reset();
            motorBurstIndex = mi;
        }

        // GP2: servos (edge trigger)
        int si = getMotorButtonIndex(gamepad2.a, gamepad2.b, gamepad2.x, gamepad2.y,
                gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.dpad_left, gamepad2.dpad_right,
                gamepad2.left_bumper, gamepad2.right_bumper, gamepad2.left_trigger > 0.5, gamepad2.right_trigger > 0.5,
                gp2APrev, gp2BPrev, gp2XPrev, gp2YPrev, gp2DpadUPrev, gp2DpadDPrev, gp2DpadLPrev, gp2DpadRPrev,
                gp2LBPrev, gp2RBPrev, gp2LTPrev, gp2RTPrev);
        gp2APrev = gamepad2.a; gp2BPrev = gamepad2.b; gp2XPrev = gamepad2.x; gp2YPrev = gamepad2.y;
        gp2DpadUPrev = gamepad2.dpad_up; gp2DpadDPrev = gamepad2.dpad_down;
        gp2DpadLPrev = gamepad2.dpad_left; gp2DpadRPrev = gamepad2.dpad_right;
        gp2LBPrev = gamepad2.left_bumper; gp2RBPrev = gamepad2.right_bumper;
        gp2LTPrev = gamepad2.left_trigger > 0.5; gp2RTPrev = gamepad2.right_trigger > 0.5;

        if (si >= 0 && si < servos.size()) {
            ServoEntry e = servos.get(si);
            if (e.isCR) {
                if (crServoBurstIndex < 0) {
                    e.crServo.setPower(CRSERVO_POWER);
                    crServoBurstTimer.reset();
                    crServoBurstIndex = si;
                }
            } else {
                e.lastPosition = Math.max(0, Math.min(1, e.lastPosition + SERVO_DELTA));
                if (e.lastPosition >= 1.0) e.lastPosition = 0.0;
                e.servo.setPosition(e.lastPosition);
            }
        }

        // Telemetria
        telemetry.clear();
        telemetry.addLine("=== HardwareMap Tester ===");
        telemetry.addLine("GP1 = Motores | GP2 = Servos | 1 botão = 1 dispositivo");
        telemetry.addLine("");
        telemetry.addLine("--- Motores (GP1) ---");
        for (int i = 0; i < motors.size() && i < MAX_MOTOR_BUTTONS; i++) {
            String btn = buttonName(i);
            telemetry.addData(btn, motors.get(i).name);
        }
        if (motors.size() > MAX_MOTOR_BUTTONS) {
            telemetry.addLine("(... mais " + (motors.size() - MAX_MOTOR_BUTTONS) + " motores no config)");
        }
        telemetry.addLine("");
        telemetry.addLine("--- Servos (GP2) ---");
        for (int i = 0; i < servos.size() && i < MAX_SERVO_BUTTONS; i++) {
            String btn = buttonName(i);
            ServoEntry se = servos.get(i);
            telemetry.addData(btn, se.name + (se.isCR ? " [CR]" : ""));
        }
        if (servos.size() > MAX_SERVO_BUTTONS) {
            telemetry.addLine("(... mais " + (servos.size() - MAX_SERVO_BUTTONS) + " servos no config)");
        }
        telemetry.update();
    }

    private static String buttonName(int index) {
        switch (index) {
            case 0: return "A";
            case 1: return "B";
            case 2: return "X";
            case 3: return "Y";
            case 4: return "DpadU";
            case 5: return "DpadD";
            case 6: return "DpadL";
            case 7: return "DpadR";
            case 8: return "LB";
            case 9: return "RB";
            case 10: return "LT";
            case 11: return "RT";
            default: return "?" + index;
        }
    }

    /** Retorna índice do botão que acabou de ser pressionado (edge), ou -1. */
    private int getMotorButtonIndex(boolean a, boolean b, boolean x, boolean y,
                                    boolean du, boolean dd, boolean dl, boolean dr,
                                    boolean lb, boolean rb, boolean lt, boolean rt,
                                    boolean aPrev, boolean bPrev, boolean xPrev, boolean yPrev,
                                    boolean duPrev, boolean ddPrev, boolean dlPrev, boolean drPrev,
                                    boolean lbPrev, boolean rbPrev, boolean ltPrev, boolean rtPrev) {
        if (a && !aPrev) return 0;
        if (b && !bPrev) return 1;
        if (x && !xPrev) return 2;
        if (y && !yPrev) return 3;
        if (du && !duPrev) return 4;
        if (dd && !ddPrev) return 5;
        if (dl && !dlPrev) return 6;
        if (dr && !drPrev) return 7;
        if (lb && !lbPrev) return 8;
        if (rb && !rbPrev) return 9;
        if (lt && !ltPrev) return 10;
        if (rt && !rtPrev) return 11;
        return -1;
    }
}
