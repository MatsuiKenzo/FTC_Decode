package org.firstinspires.ftc.teamcode.drive.national.objects;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

// Esta classe implementa um sistema de direção orientado ao campo para um robô FTC.
// Ele usa quatro motores DC para o movimento e uma IMU para rastrear a orientação do robô,
public class FieldOrientedDrive {
    // Declaração dos motores DC para cada roda do robô.
    DcMotor leftFront, leftBack, rightFront, rightBack;
    // Declaração da IMU.
    IMU imu;

    public double heading;

    // Construtor da classe FieldOrientedDrive.
    // É responsável por inicializar todos os componentes de hardware.
    public FieldOrientedDrive(HardwareMap hardwareMap) {
        // Mapeia os motores DC do arquivo de configuração de hardware do robô.
        leftFront = hardwareMap.get(DcMotor.class, "FL");
        leftBack = hardwareMap.get(DcMotor.class, "BL");
        rightFront = hardwareMap.get(DcMotor.class, "FR");
        rightBack = hardwareMap.get(DcMotor.class, "BR");

        // Define a direção dos motores esquerdos para REVERSE, o que é comum para garantir que todos os motores girem na direção correta para o movimento.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        // Cria um objeto Deadline para limitar a taxa de atualização do gamepad (não usado diretamente no movimento, mas pode ser para outras funções).
        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        // Mapeia a IMU do arquivo de configuração de hardware do robô.
        imu = hardwareMap.get(IMU.class, "imu");
        // Define os parâmetros de inicialização da IMU, especificando a orientação física do hub Rev no robô.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        // Inicializa a IMU com os parâmetros definidos.
        imu.initialize(parameters);
    }

    public void movement(double lx, double ly, double rx, boolean resetIMU) {

        // Calcula o valor máximo absoluto das entradas do joystick, garantindo que nenhuma potência exceda 1.
        double max = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);

        // Calcula a potência ajustada, permitindo uma redução da velocidade máxima.

        // Se resetIMU for verdadeiro, redefine a guinada (yaw) da IMU para a orientação atual do robô.
        if (resetIMU) {
            imu.resetYaw();
        }

        // Obtém a guinada (yaw) atual do robô da IMU em radianos.
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // Ajusta as entradas do joystick para o movimento lateral (lx) e para frente/trás (ly) com base na orientação atual do robô (heading).
        // Isso permite que o robô se mova em relação ao campo, independentemente de sua rotação.
        double adjustedLx = ly * Math.sin(heading) + lx * Math.cos(heading);
        double adjustedLy = ly * Math.cos(heading) - lx * Math.sin(heading);

        // Define a potência para cada motor, combinando os movimentos ajustados e a rotação, e normalizando pelo valor máximo.
        leftFront.setPower(((adjustedLy + adjustedLx + rx) / max));
        leftBack.setPower(((adjustedLy - adjustedLx + rx) / max));
        rightFront.setPower(((adjustedLy - adjustedLx - rx) / max));
        rightBack.setPower(((adjustedLy + adjustedLx - rx) / max));
    }

    public double getHeadingDegrees(){
        return heading;
    }
}