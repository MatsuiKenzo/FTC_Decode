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
    private Pose botPose = new Pose(0,0,0);
    private Pose blueGoalPose = new Pose(0,144,0);
    private Pose redGoalPose = blueGoalPose.mirror();

    private CRServo leftServo;
    private CRServo rightServo;
    private DcMotorEx turretEncoder;
    private double TICKS_PER_DEGREE = 16948.6; //ticks per degree final

    private DcMotorEx leftFlywheel;
    private DcMotorEx rightFlywheel;

    private Servo tiltServo;
    private InterpLUT flywheelLut = new InterpLUT();
    private InterpLUT hoodLut = new InterpLUT();
    private double minDistance = 0;
    private double maxDistance = 144;
    private double distance = 0;
    public Turret(HardwareMap hardwareMap){


        flywheelLut.add(minDistance, 0);

        flywheelLut.add(maxDistance, 0);



        flywheelLut.createLUT();

        hoodLut.add(minDistance, 0);
        hoodLut.add(maxDistance, 0);
        hoodLut.createLUT();


        leftServo = hardwareMap.get(CRServo.class, "turret_left");
        rightServo = hardwareMap.get(CRServo.class, "turret_right");

        turretEncoder = hardwareMap.get(DcMotorEx.class, ConstantsConf.Nacional.TURRET_ENCODER_MOTOR_NAME);

        resetTurret();
        leftFlywheel = hardwareMap.get(DcMotorEx.class, "shooter_right");
        rightFlywheel = hardwareMap.get(DcMotorEx.class, "shooter_left");
        leftFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tiltServo = hardwareMap.get(Servo.class, "hood");
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
    //turret
    private void resetTurret(){
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public double getTurretAngleDegrees(){
        return turretEncoder.getCurrentPosition()/TICKS_PER_DEGREE;
    }

    //turret.setBotPose(follower.getPose());
    public void setBotPose(Pose pose){
        this.botPose = pose;
        this.distance = pose.distanceFrom(getGoalPose());
    }

    PIDController turretPID = new PIDController(0.006,0,0.0005);
    private void setTurretPower(double power){
        leftServo.setPower(-power);
        rightServo.setPower(-power);
    }

    private void updateTurret(){
        double targetAngleFC = Math.atan2(
                getGoalPose().getY()-this.botPose.getY(),
                getGoalPose().getX()-this.botPose.getX()
        ); //field centric
        double targetAngleRCDegrees = Math.toDegrees(targetAngleFC-this.botPose.getHeading());

        double targetAngleRCLimited = Range.clip(targetAngleRCDegrees, -90, 90);
        double power = turretPID.calculate(getTurretAngleDegrees(), targetAngleRCLimited);

        try {
            setTurretPower(power);
        } catch (Exception e) {
            setTurretPower(0);
        }
    }
    private double getHoodTarget(){
        if (distance<minDistance){
            return hoodLut.get(minDistance+1);
        }
        if (distance>maxDistance){
            return hoodLut.get(minDistance-1);
        }
        return  hoodLut.get(distance);
    }
    private double getFlywheelTarget(){
        if (distance<minDistance){
            return flywheelLut.get(minDistance+1);
        }
        if (distance>maxDistance){
            return flywheelLut.get(minDistance-1);
        }
        return  flywheelLut.get(distance);
    }

    //flywheel
    public double getShooterVelocity(){
        return leftFlywheel.getVelocity();
    }
    public void setPIDf(double p, double i, double d, double ff){
        leftFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p,i,d,ff));
        rightFlywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p,i,d,ff));
    }
    public void setShooterVelocity(int velocity){
        rightFlywheel.setVelocity(velocity);
        leftFlywheel.setVelocity(velocity);
    }
    private void updateFlywheel(){
        //setShooterVelocity((int)getFlywheelTarget());
    }

    //hood
    public void setHoodPosition(double pos){
        this.tiltServo.setPosition(pos);
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
