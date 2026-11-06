package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SimElevator extends SubsystemBase {

    // =========================================================
    // CONFIGURAÇÕES
    // =========================================================

    // Limites físicos do elevador no CAD, em metros.
    private static final double ELEVATOR_ZERO = 0.0;
    private static final double ELEVATOR_UP = 0.40;

    // Posições equivalentes do encoder real.
    private static final double ENCODER_ZERO = 0.0;
    private static double encoderUp = 1000.0;

    // =========================================================
    // CONTROLE
    // =========================================================

    private final PIDController elevatorPID = new PIDController(1.5, 0.0, 0.0);

    // Posições internas da simulação, em metros.
    private static double currentPosition = 0.0;
    private static double setPosition = 0.0;

    // =========================================================
    // CONSTRUTOR
    // =========================================================

    public SimElevator(double realUp, double kP) {
        encoderUp = realUp;
        elevatorPID.setP(kP);
    }

    // =========================================================
    // PERIODIC
    // =========================================================

    @Override
    public void periodic() {
        updatePosition();
        updateVisualization();
    }

    // =========================================================
    // POSIÇÃO
    // =========================================================

    /**
     * Define a posição desejada do elevador usando a escala do encoder.
     */
    public void setPosition(double position, double kP) {
        elevatorPID.setP(kP);
        setPosition = map(position, ENCODER_ZERO, encoderUp, ELEVATOR_ZERO, ELEVATOR_UP);
    }

    /**
     * Retorna a posição física do elevador no CAD, em metros.
     */
    public static double getRawPosition() {
        return currentPosition;
    }

    /**
     * Retorna a posição do elevador convertida para a escala do encoder.
     */
    public static double getRealPosition() {
        return map(currentPosition, ELEVATOR_ZERO, ELEVATOR_UP, ENCODER_ZERO, encoderUp);
    }

    private void updatePosition() {
        double output = elevatorPID.calculate(currentPosition, setPosition);

        currentPosition += output * 0.02;
        currentPosition = MathUtil.clamp(currentPosition, ELEVATOR_ZERO, ELEVATOR_UP);
    }

    // =========================================================
    // COMMANDS
    // =========================================================

    public Command CMDsetPosition(double position, double kP) {
        return Commands.runOnce(() -> setPosition(position, kP));
    }

    // =========================================================
    // VISUALIZAÇÃO
    // =========================================================

    private void updateVisualization() {
        Logger.recordOutput("SubSystemSim/Elevator", new Pose3d[] {
                new Pose3d(0.0, 0.0, currentPosition, new Rotation3d()) });
    }

    // =========================================================
    // UTILIDADES
    // =========================================================

    /**
     * Converte linearmente um valor de uma escala para outra.
     */
    private static double map(double value, double inputMin, double inputMax, double outputMin, double outputMax) {
        return (value - inputMin) * (outputMax - outputMin) / (inputMax - inputMin) + outputMin;
    }
}