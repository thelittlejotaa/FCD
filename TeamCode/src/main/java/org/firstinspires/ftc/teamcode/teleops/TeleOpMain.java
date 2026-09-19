package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp
public class TeleOpMain extends OpMode {

    //instanciando o seu mecanismo isolado
    private MecanumDrive drive;

    // configuracoes de pilotagem
    private static final double DEADZONE = 0.05;
    private int movementInverter = 1;
    private boolean isFieldCentric = true;
    private boolean lastBumperState = false;

    @Override
    public void init() {
        drive = new MecanumDrive();
        drive.init(hardwareMap);

        telemetry.addLine("MecanumDrive Inicializado.");
        telemetry.addLine("Odometria Pinpoint pronta.");
        telemetry.update();
    }

    @Override
    public void loop() {
        //inversao de movimento
        if (gamepad1.dpad_left) movementInverter = -1;
        if (gamepad1.dpad_down) movementInverter = 1;

        //alternar entre Field Centric e Robot Centric (gatilho da direita)
        if (gamepad1.right_bumper && !lastBumperState) {
            isFieldCentric = !isFieldCentric;
        }
        lastBumperState = gamepad1.right_bumper;

        // aplicação de zona morta (elimina arrasto quando solta o analogico)
        double rawX = gamepad1.left_stick_x;
        double rawY = gamepad1.left_stick_y;
        double rawTurn = gamepad1.right_stick_x;

        double x = (Math.abs(rawX) > DEADZONE) ? rawX : 0.0;
        double y = (Math.abs(rawY) > DEADZONE) ? rawY : 0.0;
        double turn = (Math.abs(rawTurn) > DEADZONE) ? rawTurn : 0.0;

        // suavização cubica e Inversão
        //ele eleva ao cubo para dar movimentos finos quando o analogico é pouco pressionado, mas mantem a velocidade maxima no fim do curso
        x = Math.pow(x, 3) * movementInverter;
        y = Math.pow(y, 3) * movementInverter;
        turn = Math.pow(turn, 3);

        // envio dos comandos para o mecanismo (O botao Start reseta o norte do IMU/pinpoint)
        if (isFieldCentric) {
            drive.moveFieldCentric(x, y, turn, gamepad1.start);
        } else {
            drive.moveDriveTrain(x, y, turn);
        }

        telemetry.addData("--- PILOTAGEM ---", "");
        telemetry.addData("Modo", isFieldCentric ? "FIELD CENTRIC" : "ROBOT CENTRIC");
        telemetry.addData("Direção", movementInverter == 1 ? "Normal" : "Invertida");
        telemetry.addData("Eixos", "X: %.2f | Y: %.2f | Giro: %.2f", x, y, turn);

        telemetry.addData("--- DIAGNÓSTICO DO HARDWARE ---", "");
        telemetry.addData("Correntes (A)", drive.getCurrents());
        telemetry.addData("Potências", drive.getPowers());
        telemetry.addData("Encoders", drive.getPositions());

        telemetry.update();
    }
}