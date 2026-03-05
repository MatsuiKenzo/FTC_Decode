package org.firstinspires.ftc.teamcode.drive.national.subsystems;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.drive.util.ConstantsConf;

public class Turret {
    /** Convenção igual à NacionalTurret: 0° = costas do robô. */
    private static final double TURRET_ANGLE_OFFSET_DEG = 180.0;

    private Pose botPose = new Pose(0, 0, 0);
    private Pose blueGoalPose = new Pose(0, 144, 0);
    private Pose redGoalPose = blueGoalPose.mirror();

    private CRServo leftServo;
    private CRServo rightServo;
    private DcMotorEx turretEncoder;
    private int encoderZeroPosition = 0;
    private double encoderTicksPerTurretRev = 0;
    private double encoderDirection = 1.0;

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;

    private Servo tiltServo;
    private InterpLUT flywheelLut = new InterpLUT();
    private InterpLUT hoodLut = new InterpLUT();
    private double minDistance = 0;
    private double maxDistance = 144;
    private double distance = 0;

    public Turret(HardwareMap hardwareMap) {
        flywheelLut.add(minDistance, 0);
        flywheelLut.add(maxDistance, 0);
        flywheelLut.createLUT();

        hoodLut.add(minDistance, 0);
        hoodLut.add(maxDistance, 0);
        hoodLut.createLUT();

        leftServo = hardwareMap.get(CRServo.class, ConstantsConf.Nacional.TURRET_LEFT_SERVO_NAME);
        rightServo = hardwareMap.get(CRServo.class, ConstantsConf.Nacional.TURRET_RIGHT_SERVO_NAME);

        if (ConstantsConf.Nacional.TURRET_ENCODER_ENABLED) {
            try {
                turretEncoder = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.TURRET_ENCODER_MOTOR_NAME);
                turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                encoderZeroPosition = turretEncoder.getCurrentPosition();
                int ticksPerRev = ConstantsConf.Nacional.TURRET_ENCODER_TICKS_PER_REV;
                int turretTeeth = ConstantsConf.Nacional.TURRET_ENCODER_GEAR_TURRET_TEETH;
                int encoderTeeth = ConstantsConf.Nacional.TURRET_ENCODER_GEAR_ENCODER_TEETH;
                encoderTicksPerTurretRev = (double) ticksPerRev * turretTeeth / encoderTeeth;
                encoderDirection = ConstantsConf.Nacional.TURRET_ENCODER_DIRECTION;
            } catch (Exception e) {
                turretEncoder = null;
            }
        } else {
            turretEncoder = null;
        }

        leftFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_RIGHT_MOTOR_NAME);
        rightFlywheel = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.SHOOTER_LEFT_MOTOR_NAME);
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tiltServo = hardwareMap.get(Servo.class, ConstantsConf.Nacional.HOOD_SERVO_NAME);
        tiltServo.setPosition(1.0);
    }

    public double getDistanceToGoal() {
        return distance;
    }

    //side
    public enum SIDES{
        BLUE,
        RED
    }
    private SIDES side = SIDES.BLUE;
    public void setSide(SIDES side){
        this.side = side;
    }
    private Pose getGoalPose(){
        return side==SIDES.RED?redGoalPose:blueGoalPose;
    }
    /** Normaliza ângulo para [-180, 180]. */
    private static double normalizeAngle(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }

    /** Ângulo da turret em graus (convenção 0° = costas do robô). Retorna 0 se encoder não estiver disponível. */
    public double getTurretAngleDegrees() {
        if (turretEncoder == null || encoderTicksPerTurretRev <= 0) return 0.0;
        int raw = turretEncoder.getCurrentPosition() - encoderZeroPosition;
        double encoderDeg = (raw * 360.0 / encoderTicksPerTurretRev) * encoderDirection;
        return normalizeAngle(encoderDeg - TURRET_ANGLE_OFFSET_DEG);
    }

    //turret.setBotPose(follower.getPose());
    public void setBotPose(Pose pose){
        this.botPose = pose;
        this.distance = pose.distanceFrom(getGoalPose());
    }

    PIDController turretPID = new PIDController(0.1, 0, 0);

    private void setTurretPower(double power) {
        if (leftServo != null) leftServo.setPower(-power);
        if (rightServo != null) rightServo.setPower(-power);
    }

    private void updateTurret() {
        if (turretEncoder == null) return;
        double targetAngleFC = Math.atan2(
                getGoalPose().getY() - this.botPose.getY(),
                getGoalPose().getX() - this.botPose.getX()
        );
        // Robot-centric: 0° = frente do robô
        double targetAngleRCDegrees = Math.toDegrees(targetAngleFC - this.botPose.getHeading());
        double targetAngleRCLimited = Range.clip(targetAngleRCDegrees, -90, 90);
        // Converter para convenção 0° = costas (igual NacionalTurret)
        double targetCostas = normalizeAngle(targetAngleRCLimited + TURRET_ANGLE_OFFSET_DEG);
        double current = getTurretAngleDegrees();
        // Erro pelo caminho mais curto: evita girar 350° quando bastam 10°
        double errorDeg = normalizeAngle(targetCostas - current);
        // PID com setpoint 0 e "medida" = -error para que o erro interno seja errorDeg
        double power = turretPID.calculate(-errorDeg, 0.0);
        power = Range.clip(power, -1.0, 1.0);
        setTurretPower(power);
    }
    private double getHoodTarget() {
        double d = Range.clip(distance, minDistance, maxDistance);
        return hoodLut.get(d);
    }

    private double getFlywheelTarget() {
        double d = Range.clip(distance, minDistance, maxDistance);
        return flywheelLut.get(d);
    }

    // flywheel
    public double getShooterVelocity() {
        if (leftFlywheel != null) return leftFlywheel.getVelocity();
        return 0.0;
    }

    public void setPIDf(double p, double i, double d, double ff) {
        PIDFCoefficients coeffs = new PIDFCoefficients((float) p, (float) i, (float) d, (float) ff);
        if (leftFlywheel != null) leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        if (rightFlywheel != null) rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
    }

    public void setShooterVelocity(int velocity) {
        if (rightFlywheel != null) rightFlywheel.setVelocity(velocity);
        if (leftFlywheel != null) leftFlywheel.setVelocity(velocity);
    }
    private void updateFlywheel(){
        //setShooterVelocity((int)getFlywheelTarget());
    }

    // hood
    public void setHoodPosition(double pos) {
        pos = Range.clip(pos, 0.0, 1.0);
        if (tiltServo != null) tiltServo.setPosition(pos);
    }

    private void updateHood(){
        //setHoodPosition(getHoodTarget());
    }
    public void periodic(){
        updateHood();
        updateFlywheel();
        updateTurret();
    }

}
