package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;
import java.util.List;

public class MecanumDrive {
    // R e L sao right e left B e F sao back e front
    public DcMotorEx RFMotor;
    public DcMotorEx LFMotor;
    public DcMotorEx RBMotor;
    public DcMotorEx LBMotor;

    // adiciona o computador de odometria Pinpoint
    public GoBildaPinpointDriver odo;

    public void init(HardwareMap hwmap){
        RFMotor = (DcMotorEx) hwmap.get(DcMotor.class, "RFMotor");
        LFMotor = (DcMotorEx) hwmap.get(DcMotor.class, "LFMotor");
        RBMotor = (DcMotorEx) hwmap.get(DcMotor.class, "RBMotor");
        LBMotor = (DcMotorEx) hwmap.get(DcMotor.class, "LBMotor");

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // inicializa e configura o Pinpoint
        odo = hwmap.get(GoBildaPinpointDriver.class, "odo");

        // Configuracoes fisicas (medir e ajustar esses offsets depois)
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // reseta o ponto inicial da arena
        odo.resetPosAndIMU();
        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);
    }

    // NOVA FUNCAO: move o robO usando as coordenadas globais do campo (Field Centric)
    public void moveFieldCentric(double gPad_lsx, double gPad_lsy, double gPad_rsx, boolean resetIMU) {
        // atualiza as leituras do sensor
        odo.update();

        // botao para resetar o norte
        if (resetIMU) {
            odo.resetPosAndIMU();
        }

        // Coleta os valores do gamepad
        double x = gPad_lsx;
        double y = -gPad_lsy;
        double turn = gPad_rsx;

        // pega o angulo atual que o robo ta apontando direto do Pinpoint
        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        // oordenadas polares
        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        //  FIELD CENTRIC: rotaciona o angulo do analogico baseado na rotacao do robo
        // subtraimos o 'heading' para compensar a orientacao atual do chassi no campo
        double targetTheta = theta - heading - (Math.PI / 4);

        double sin = Math.sin(targetTheta);
        double cos = Math.cos(targetTheta);

        // correcao de circulo para quadrado
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        // distribuicao de potencia com base no vetor rotacionado
        double leftFront  = power * (cos / max) + turn;
        double rightFront = power * (sin / max) - turn;
        double leftRear   = power * (sin / max) + turn;
        double rightRear  = power * (cos / max) - turn;

        // protecao para nao passar de 1.0
        if ((power + Math.abs(turn)) > 1) {
            leftFront  /= (power + Math.abs(turn));
            rightFront /= (power + Math.abs(turn));
            leftRear   /= (power + Math.abs(turn));
            rightRear  /= (power + Math.abs(turn));
        }

        LFMotor.setPower(leftFront);
        RFMotor.setPower(rightFront);
        LBMotor.setPower(leftRear);
        RBMotor.setPower(rightRear);
    }

    // mantem o metodo original para testar em modo robot centric
    public void moveDriveTrain(double gPad_lsx, double gPad_lsy, double gPad_rsx) {
        double x = gPad_lsx;
        double y = -gPad_lsy;
        double turn = gPad_rsx;

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);

        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFront  = power * (cos / max) + turn;
        double rightFront = power * (sin / max) - turn;
        double leftRear   = power * (sin / max) + turn;
        double rightRear  = power * (cos / max) - turn;

        if ((power + Math.abs(turn)) > 1) {
            leftFront  /= (power + Math.abs(turn));
            rightFront /= (power + Math.abs(turn));
            leftRear   /= (power + Math.abs(turn));
            rightRear  /= (power + Math.abs(turn));
        }

        LFMotor.setPower(leftFront);
        RFMotor.setPower(rightFront);
        LBMotor.setPower(leftRear);
        RBMotor.setPower(rightRear);
    }

    public void setMotorModes(DcMotor.RunMode mode) {
        LFMotor.setMode(mode);
        RFMotor.setMode(mode);
        LBMotor.setMode(mode);
        RBMotor.setMode(mode);
    }

    public void stopMotors() {
        LFMotor.setPower(0);
        RFMotor.setPower(0);
        LBMotor.setPower(0);
        RBMotor.setPower(0);
    }

    public void setTargetPositions(int lf, int rf, int lb, int rb) {
        LFMotor.setTargetPosition(lf);
        RFMotor.setTargetPosition(rf);
        LBMotor.setTargetPosition(lb);
        RBMotor.setTargetPosition(rb);
    }

    public void setPowers(double lfmPower, double rfmPower, double lbmPower, double rbmPower){
        LFMotor.setPower(lfmPower);
        RFMotor.setPower(rfmPower);
        LBMotor.setPower(lbmPower);
        RBMotor.setPower(rbmPower);
    }

    public String getCurrents(){
        List<Double> currents = new ArrayList<>();
        currents.add(LFMotor.getCurrent(CurrentUnit.AMPS));
        currents.add(RFMotor.getCurrent(CurrentUnit.AMPS));
        currents.add(LBMotor.getCurrent(CurrentUnit.AMPS));
        currents.add(RBMotor.getCurrent(CurrentUnit.AMPS));
        return currents.toString();
    }

    public String getPowers(){
        List<Double> powers = new ArrayList<>();
        powers.add(LFMotor.getPower());
        powers.add(RFMotor.getPower());
        powers.add(LBMotor.getPower());
        powers.add(RBMotor.getPower());
        return powers.toString();
    }
    public String getPositions(){
        List<Integer> positions = new ArrayList<>();
        positions.add(LFMotor.getCurrentPosition());
        positions.add(RFMotor.getCurrentPosition());
        positions.add(LBMotor.getCurrentPosition());
        positions.add(RBMotor.getCurrentPosition());
        return positions.toString();
    }

    public boolean LFMotorIsBusy(){ return LFMotor.isBusy(); }
    public boolean RFMotorIsBusy(){ return RFMotor.isBusy(); }
    public boolean LBMotorIsBusy(){ return LBMotor.isBusy(); }
    public boolean RBMotorIsBusy(){ return RBMotor.isBusy(); }
}